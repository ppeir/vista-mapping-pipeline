import { useState, useEffect, useRef } from 'react'

// ── Types ─────────────────────────────────────────────────────────────────

type RecordStatus = 'idle' | 'recording' | 'done' | 'error'
type PipelineStatus = 'idle' | 'running' | 'done' | 'error'

interface PipelinePayload {
  config: string
  svo_stem: string
  map_choice: 1 | 2
  extra_args: string[]
}

interface ParamDef {
  key: string
  cli: string
  type: string   // "float" | "int" | "str" | "bool" | "choice:a,b,c"
  label: string
  value: string  // preset default as string
}

// ── API helpers ───────────────────────────────────────────────────────────

async function apiGet<T>(path: string): Promise<T> {
  const r = await fetch(path)
  if (!r.ok) throw new Error(`${r.status} ${r.statusText}`)
  return r.json() as Promise<T>
}

async function apiPost<T>(path: string, body?: unknown): Promise<T> {
  const r = await fetch(path, {
    method: 'POST',
    headers: body !== undefined ? { 'Content-Type': 'application/json' } : {},
    body: body !== undefined ? JSON.stringify(body) : undefined,
  })
  if (!r.ok) {
    const msg = await r.text().catch(() => r.statusText)
    throw new Error(msg)
  }
  return r.json() as Promise<T>
}

// ── Helpers ───────────────────────────────────────────────────────────────

function fmtTime(s: number): string {
  const m = Math.floor(s / 60)
  const sec = s % 60
  return `${String(m).padStart(2, '0')}:${String(sec).padStart(2, '0')}`
}

// ── Small UI primitives ───────────────────────────────────────────────────

function Input({
  label,
  value,
  onChange,
  placeholder,
  disabled,
  type = 'text',
}: {
  label: string
  value: string
  onChange: (v: string) => void
  placeholder?: string
  disabled?: boolean
  type?: string
}) {
  return (
    <div className="space-y-1">
      <label className="block text-xs text-gray-400">{label}</label>
      <input
        type={type}
        step="any"
        className="w-full bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm
                   focus:outline-none focus:border-blue-500 disabled:opacity-40"
        placeholder={placeholder}
        value={value}
        onChange={e => onChange(e.target.value)}
        disabled={disabled}
      />
    </div>
  )
}

function Select({
  label,
  value,
  onChange,
  options,
  disabled,
}: {
  label: string
  value: string
  onChange: (v: string) => void
  options: { value: string; label: string }[]
  disabled?: boolean
}) {
  return (
    <div className="space-y-1">
      <label className="block text-xs text-gray-400">{label}</label>
      <select
        className="w-full bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm
                   focus:outline-none focus:border-blue-500 disabled:opacity-40"
        value={value}
        onChange={e => onChange(e.target.value)}
        disabled={disabled}
      >
        {options.map(o => (
          <option key={o.value} value={o.value}>{o.label}</option>
        ))}
      </select>
    </div>
  )
}

// ── Main component ────────────────────────────────────────────────────────

export default function App() {
  // ── Capture state ──────────────────────────────────────────────
  const [sessionName, setSessionName] = useState('')
  const [fps, setFps] = useState('15')
  const [imuWarmup, setImuWarmup] = useState('2.0')
  const [captureWait, setCaptureWait] = useState('0')
  const [showCaptureAdvanced, setShowCaptureAdvanced] = useState(false)
  const [recordStatus, setRecordStatus] = useState<RecordStatus>('idle')
  const [frameCount, setFrameCount] = useState(0)
  const [elapsed, setElapsed] = useState(0)
  const [recordLogs, setRecordLogs] = useState<string[]>([])
  const [recordSseUrl, setRecordSseUrl] = useState<string | null>(null)
  const startTimeRef = useRef<number>(0)
  const timerRef = useRef<ReturnType<typeof setInterval> | null>(null)
  const recordLogEndRef = useRef<HTMLDivElement>(null)

  // ── Pipeline state ─────────────────────────────────────────────
  const [configs, setConfigs] = useState<string[]>([])
  const [svos, setSvos] = useState<string[]>([])
  const [selectedConfig, setSelectedConfig] = useState('')
  const [selectedSvo, setSelectedSvo] = useState('')
  const [mapChoice, setMapChoice] = useState<'1' | '2'>('1')
  const [presetParams, setPresetParams] = useState<ParamDef[]>([])
  const [paramValues, setParamValues] = useState<Record<string, string>>({})
  const [showAdvanced, setShowAdvanced] = useState(false)
  const [pipelineStatus, setPipelineStatus] = useState<PipelineStatus>('idle')
  const [logs, setLogs] = useState<string[]>([])
  const [zipReady, setZipReady] = useState(false)
  const [pipelineSseUrl, setPipelineSseUrl] = useState<string | null>(null)
  const logEndRef = useRef<HTMLDivElement>(null)

  // ── Load configs and restore process states on mount ───────────
  const fetchSvos = () =>
    apiGet<{ svos: string[] }>('/api/svos')
      .then(d => { setSvos(d.svos); if (d.svos.length > 0 && !selectedSvo) setSelectedSvo(d.svos[0]) })
      .catch(console.error)

  useEffect(() => {
    apiGet<{ configs: string[] }>('/api/configs')
      .then(d => { setConfigs(d.configs); if (d.configs.length > 0) setSelectedConfig(d.configs[0]) })
      .catch(console.error)

    fetchSvos()

    apiGet<{ record: string; pipeline: string }>('/api/status')
      .then(s => {
        if (s.record === 'running') {
          setRecordStatus('recording')
          setRecordSseUrl(`/api/logs/record?t=${Date.now()}`)
        } else if (s.record === 'done') setRecordStatus('done')
        else if (s.record === 'error') setRecordStatus('error')

        if (s.pipeline === 'running') {
          setPipelineStatus('running')
          setPipelineSseUrl(`/api/logs/pipeline?t=${Date.now()}`)
        } else if (s.pipeline === 'done') setPipelineStatus('done')
        else if (s.pipeline === 'error') setPipelineStatus('error')
      })
      .catch(console.error)
  }, [])

  // ── Fetch preset params when config changes ────────────────────
  useEffect(() => {
    if (!selectedConfig) { setPresetParams([]); setParamValues({}); return }
    apiGet<{ params: ParamDef[] }>(`/api/presets/${selectedConfig}`)
      .then(d => {
        setPresetParams(d.params)
        const defaults: Record<string, string> = {}
        for (const p of d.params) defaults[p.key] = p.value
        setParamValues(defaults)
      })
      .catch(console.error)
  }, [selectedConfig])

  // ── Auto-scroll logs ───────────────────────────────────────────
  useEffect(() => {
    recordLogEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [recordLogs])

  useEffect(() => {
    logEndRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [logs])

  // ── SSE: record ────────────────────────────────────────────────
  useEffect(() => {
    if (!recordSseUrl) return
    const sse = new EventSource(recordSseUrl)
    sse.onmessage = (e: MessageEvent<string>) => {
      const data = e.data
      if (data.startsWith('[DONE]')) {
        const m = data.match(/exit=(-?\d+)/)
        const ok = m && m[1] === '0'
        setRecordStatus(ok ? 'done' : 'error')
        setRecordLogs(prev => [...prev, ok ? '[OK] Recording done.' : `[ERROR] Exit code ${m?.[1]}.`])
        setRecordSseUrl(null)
        if (timerRef.current) clearInterval(timerRef.current)
        sse.close()
      } else {
        const fm = data.match(/Frame count:\s*(\d+)/i)
        if (fm) {
          const count = parseInt(fm[1])
          setFrameCount(count)
          // Start the timer on the very first frame
          if (count === 1 && !timerRef.current) {
            startTimeRef.current = Date.now()
            timerRef.current = setInterval(
              () => setElapsed(Math.floor((Date.now() - startTimeRef.current) / 1000)),
              1000,
            )
          }
        } else if (data && !data.startsWith(':')) {
          setRecordLogs(prev => [...prev, data])
        }
      }
    }
    sse.onerror = () => sse.close()
    return () => sse.close()
  }, [recordSseUrl])

  // ── SSE: pipeline ──────────────────────────────────────────────
  useEffect(() => {
    if (!pipelineSseUrl) return
    const sse = new EventSource(pipelineSseUrl)
    sse.onmessage = (e: MessageEvent<string>) => {
      const data = e.data
      if (data.startsWith('[DONE]')) {
        const m = data.match(/exit=(-?\d+)/)
        const ok = m && m[1] === '0'
        setPipelineStatus(ok ? 'done' : 'error')
        if (ok) setZipReady(true)
        setPipelineSseUrl(null)
        sse.close()
      } else if (data && !data.startsWith(':')) {
        setLogs(prev => [...prev, data])
      }
    }
    sse.onerror = () => sse.close()
    return () => sse.close()
  }, [pipelineSseUrl])

  // ── Handlers ───────────────────────────────────────────────────

  const handleStartRecording = async () => {
    if (!sessionName.trim()) { alert('A session name is required'); return }
    try {
      await apiPost('/api/record/start', {
        session_name: sessionName.trim(),
        fps: parseInt(fps),
        imu_warmup: isNaN(parseFloat(imuWarmup)) ? 2.0 : parseFloat(imuWarmup),
        wait: isNaN(parseInt(captureWait)) ? 0 : parseInt(captureWait),
      })
      setRecordStatus('recording')
      setFrameCount(0)
      setElapsed(0)
      setRecordLogs([])
      timerRef.current = null
      setRecordSseUrl(`/api/logs/record?t=${Date.now()}`)
    } catch (err) {
      alert(`Failed to start recording: ${err instanceof Error ? err.message : err}`)
    }
  }

  const handleStopRecording = async () => {
    try {
      await apiPost('/api/record/stop')
      if (timerRef.current) clearInterval(timerRef.current)
    } catch (err) {
      alert(`Failed to stop recording: ${err instanceof Error ? err.message : err}`)
    }
  }

  const handleKillPipeline = async () => {
    try {
      await apiPost('/api/pipeline/stop')
    } catch (err) {
      alert(`Failed to kill pipeline: ${err instanceof Error ? err.message : err}`)
    }
  }

  const handleStartPipeline = async () => {
    if (!selectedConfig) { alert('Sélectionner un preset'); return }
    if (!selectedSvo) { alert('Sélectionner un fichier SVO2'); return }
    setLogs([])
    setZipReady(false)
    try {
      const extra_args: string[] = []
      for (const p of presetParams) {
        const val = paramValues[p.key] ?? p.value
        if (p.type === 'bool') {
          if (p.key === 'depth') {
            extra_args.push(val === 'true' ? '--depth' : '--no-depth')
          } else if (val === 'true') {
            extra_args.push(p.cli)
          }
          // false flags are simply omitted
        } else {
          extra_args.push(p.cli, val)
        }
      }
      const payload: PipelinePayload = {
        config: selectedConfig,
        svo_stem: selectedSvo,
        map_choice: mapChoice === '1' ? 1 : 2,
        extra_args,
      }
      await apiPost('/api/pipeline/start', payload)
      setPipelineStatus('running')
      setPipelineSseUrl(`/api/logs/pipeline?t=${Date.now()}`)
    } catch (err) {
      alert(`Failed to start pipeline: ${err instanceof Error ? err.message : err}`)
    }
  }

  // ── Derived ────────────────────────────────────────────────────
  const isRecording = recordStatus === 'recording'
  const isPipelineRunning = pipelineStatus === 'running'
  const zipSession = selectedSvo

  // ── Render ─────────────────────────────────────────────────────
  return (
    <div className="min-h-screen bg-gray-950 text-gray-100">
      {/* Header */}
      <header className="bg-gray-900 border-b border-gray-800 px-6 py-4">
        <div className="max-w-7xl mx-auto flex items-baseline gap-3">
          <h1 className="text-xl font-bold tracking-tight text-white">Vista Capture App</h1>
          <span className="text-sm text-gray-500">ZED2i · RTAB-Map · Jetson Orin</span>
        </div>
      </header>

      {/* Two-panel layout */}
      <main className="p-6 grid grid-cols-1 lg:grid-cols-2 gap-6 max-w-7xl mx-auto">

        {/* ──────────────────── Capture Panel ──────────────────── */}
        <section className="bg-gray-900 rounded-2xl border border-gray-800 p-6 space-y-5">
          <div className="flex items-center justify-between">
            <h2 className="text-lg font-semibold">Capture</h2>
            {isRecording && (
              <span className="flex items-center gap-2 text-sm text-red-400 font-medium">
                <span className="w-2 h-2 rounded-full bg-red-500 animate-pulse" />
                RECORDING
              </span>
            )}
            {recordStatus === 'done' && <span className="text-sm text-green-400">✓ Done</span>}
            {recordStatus === 'error' && <span className="text-sm text-red-400">✗ Error</span>}
          </div>

          <Input
            label="Session name"
            value={sessionName}
            onChange={setSessionName}
            placeholder="Session name"
            disabled={isRecording}
          />

          <Select
            label="FPS"
            value={fps}
            onChange={setFps}
            disabled={isRecording}
            options={[
              { value: '15', label: '15 fps' },
              { value: '30', label: '30 fps' },
              { value: '60', label: '60 fps' },
            ]}
          />

          {/* Paramètres avancés capture */}
          <div>
            <button
              onClick={() => setShowCaptureAdvanced(v => !v)}
              disabled={isRecording}
              className="flex items-center gap-1.5 text-sm text-gray-400 hover:text-gray-200 transition-colors disabled:opacity-40"
            >
              <span>{showCaptureAdvanced ? '▾' : '▸'}</span>
              <span>Paramètres avancés</span>
            </button>
            {showCaptureAdvanced && (
              <div className="mt-3 grid grid-cols-2 gap-3">
                <Input
                  label="Warmup IMU (s)"
                  value={imuWarmup}
                  onChange={setImuWarmup}
                  disabled={isRecording}
                  type="number"
                  placeholder="2.0"
                />
                <Input
                  label="Délai démarrage (s)"
                  value={captureWait}
                  onChange={setCaptureWait}
                  disabled={isRecording}
                  type="number"
                  placeholder="0"
                />
              </div>
            )}
          </div>

          {/* Live stats while recording */}
          {isRecording && (
            <div className="bg-gray-800 rounded-xl px-5 py-4 space-y-1">
              <div className="text-3xl font-mono tabular-nums tracking-tight">
                {frameCount > 0 ? fmtTime(elapsed) : '––:––'}
              </div>
              <div className="text-sm text-gray-400">
                {frameCount > 0 ? `${frameCount.toLocaleString()} frames` : 'Initialisation…'}
              </div>
            </div>
          )}

          {!isRecording ? (
            <button
              onClick={handleStartRecording}
              className="w-full bg-blue-600 hover:bg-blue-500 rounded-xl py-2.5 text-sm
                         font-medium transition-colors"
            >
              Start Recording
            </button>
          ) : (
            <button
              onClick={handleStopRecording}
              className="w-full bg-red-700 hover:bg-red-600 rounded-xl py-2.5 text-sm
                         font-medium transition-colors"
            >
              Stop Recording
            </button>
          )}

          {/* Record log console */}
          {recordLogs.length > 0 && (
            <div className="log-console bg-gray-950 border border-gray-800 rounded-xl p-3 h-40
                            overflow-y-auto font-mono text-xs leading-relaxed">
              {recordLogs.map((line, i) => (
                <div
                  key={i}
                  className={
                    /\[ERROR\]|error|Error/i.test(line)
                      ? 'text-red-400'
                      : /\[OK\]/i.test(line)
                      ? 'text-green-400'
                      : 'text-gray-300'
                  }
                >
                  {line}
                </div>
              ))}
              <div ref={recordLogEndRef} />
            </div>
          )}
        </section>

        {/* ──────────────────── Pipeline Panel ─────────────────── */}
        <section className="bg-gray-900 rounded-2xl border border-gray-800 p-6 space-y-5">
          <div className="flex items-center justify-between">
            <h2 className="text-lg font-semibold">Pipeline</h2>
            {isPipelineRunning && (
              <span className="flex items-center gap-2 text-sm text-yellow-400 font-medium">
                <span className="w-2 h-2 rounded-full bg-yellow-400 animate-pulse" />
                Running
              </span>
            )}
            {pipelineStatus === 'done' && <span className="text-sm text-green-400">✓ Done</span>}
            {pipelineStatus === 'error' && <span className="text-sm text-red-400">✗ Error</span>}
          </div>

          {/* SVO2 file dropdown */}
          <div className="space-y-1">
            <label className="block text-xs text-gray-400">Fichier SVO2</label>
            <select
              className="w-full bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm
                         focus:outline-none focus:border-blue-500 disabled:opacity-40"
              value={selectedSvo}
              onFocus={fetchSvos}
              onChange={e => setSelectedSvo(e.target.value)}
              disabled={isPipelineRunning}
            >
              <option value="">— select —</option>
              {svos.map(s => <option key={s} value={s}>{s}.svo2</option>)}
            </select>
          </div>

          {/* Preset dropdown */}
          <div className="space-y-1">
            <label className="block text-xs text-gray-400">Preset</label>
            <select
              className="w-full bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm
                         focus:outline-none focus:border-blue-500 disabled:opacity-40"
              value={selectedConfig}
              onChange={e => setSelectedConfig(e.target.value)}
              disabled={isPipelineRunning}
            >
              <option value="">— select —</option>
              {configs.map(c => <option key={c} value={c}>{c.charAt(0).toUpperCase() + c.slice(1)}</option>)}
            </select>
          </div>

          {/* Map choice (always visible) */}
          <Select
            label="Map choice"
            value={mapChoice}
            onChange={v => setMapChoice(v as '1' | '2')}
            disabled={isPipelineRunning}
            options={[
              { value: '1', label: 'RTAB-Map projection' },
              { value: '2', label: 'Manual projection' },
            ]}
          />

          {/* Paramètres avancés — dynamic, collapsible */}
          {presetParams.length > 0 && (
            <div>
              <button
                onClick={() => setShowAdvanced(v => !v)}
                disabled={isPipelineRunning}
                className="flex items-center gap-1.5 text-sm text-gray-400 hover:text-gray-200 transition-colors disabled:opacity-40"
              >
                <span>{showAdvanced ? '▾' : '▸'}</span>
                <span>Paramètres avancés</span>
              </button>
              {showAdvanced && (
                <div className="mt-3 grid grid-cols-2 gap-3">
                  {presetParams.map(p => {
                    const cur = paramValues[p.key] ?? p.value
                    if (p.type === 'bool') {
                      return (
                        <div key={p.key} className="flex items-center gap-2 col-span-1">
                          <input
                            type="checkbox"
                            id={`param-${p.key}`}
                            checked={cur === 'true'}
                            onChange={e => setParamValues(prev => ({ ...prev, [p.key]: e.target.checked ? 'true' : 'false' }))}
                            disabled={isPipelineRunning}
                            className="h-4 w-4 rounded border-gray-600 bg-gray-800 accent-blue-500 disabled:opacity-40"
                          />
                          <label htmlFor={`param-${p.key}`} className="text-xs text-gray-300">{p.label}</label>
                        </div>
                      )
                    }
                    if (p.type.startsWith('choice:')) {
                      const choices = p.type.slice(7).split(',')
                      return (
                        <Select
                          key={p.key}
                          label={p.label}
                          value={cur}
                          onChange={v => setParamValues(prev => ({ ...prev, [p.key]: v }))}
                          disabled={isPipelineRunning}
                          options={choices.map(c => ({ value: c, label: c }))}
                        />
                      )
                    }
                    return (
                      <Input
                        key={p.key}
                        label={p.label}
                        value={cur}
                        onChange={v => setParamValues(prev => ({ ...prev, [p.key]: v }))}
                        disabled={isPipelineRunning}
                        type={p.type === 'float' || p.type === 'int' ? 'number' : 'text'}
                        placeholder={p.value}
                      />
                    )
                  })}
                </div>
              )}
            </div>
          )}

          {/* Launch / Kill buttons */}
          <div className="flex gap-2">
            <button
              onClick={handleStartPipeline}
              disabled={isPipelineRunning || !selectedConfig || !selectedSvo}
              className="flex-1 bg-green-700 hover:bg-green-600 disabled:bg-gray-700 rounded-xl
                         py-2.5 text-sm font-medium transition-colors"
            >
              {isPipelineRunning ? 'Processing…' : 'Launch Pipeline'}
            </button>
            {isPipelineRunning && (
              <button
                onClick={handleKillPipeline}
                className="bg-red-800 hover:bg-red-700 rounded-xl px-4 text-sm font-medium transition-colors"
              >
                Kill
              </button>
            )}
          </div>

          {/* Log console */}
          {logs.length > 0 && (
            <div className="log-console bg-gray-950 border border-gray-800 rounded-xl p-3 h-60
                            overflow-y-auto font-mono text-xs leading-relaxed">
              {logs.map((line, i) => (
                <div
                  key={i}
                  className={
                    /\[ERROR\]|error|Error/.test(line)
                      ? 'text-red-400'
                      : /\[OK\]|\[DONE\]|Done/i.test(line)
                      ? 'text-green-400'
                      : /Step \d/.test(line)
                      ? 'text-blue-300 font-semibold'
                      : 'text-gray-300'
                  }
                >
                  {line}
                </div>
              ))}
              <div ref={logEndRef} />
            </div>
          )}

          {/* Download ZIP */}
          {zipReady && zipSession && (
            <a
              href={`/api/download/${encodeURIComponent(zipSession)}`}
              download
              className="flex items-center justify-center gap-2 w-full bg-indigo-700
                         hover:bg-indigo-600 rounded-xl py-2.5 text-sm font-medium
                         transition-colors"
            >
              ↓ Download {zipSession}.zip
            </a>
          )}
        </section>
      </main>
    </div>
  )
}
