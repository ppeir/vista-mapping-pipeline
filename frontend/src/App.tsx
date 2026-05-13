import { useState, useEffect, useRef } from 'react'
import toast, { Toaster } from 'react-hot-toast'

// ── Types ─────────────────────────────────────────────────────────────────

type RecordStatus = 'idle' | 'recording' | 'done' | 'error'
type PipelineStatus = 'idle' | 'running' | 'done' | 'error'

interface PipelinePayload {
  config: string
  svo_stem: string
  output_name: string
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

// ── Pipeline step groups ──────────────────────────────────────────────────

const GLOBAL_PARAM_KEYS = new Set(['trim_start', 'trim_end', 'depth_scale'])

interface StepGroup {
  id: string
  label: string
  keys: Set<string>
  includeRtabmap?: boolean
  skipFlag?: string | null
}

const EASTER_EGG_MESSAGES = [
  '🚀 Pipeline successfully completed in 0.0001 ms! New personal best. (Tip: try checking at least one box next time).',
  'Request to process the void received. The robot thought long and hard about nothing, and absolutely loved it.',
  'No steps selected. I used this free time to meditate on the meaning of life and sort my bits.',
  'Error 404: Intention to work not found. Please check at least one box to wake up the processor.',
  'Executing DoNothing() algorithm... Absolute success.',
]

const STEP_GROUPS: StepGroup[] = [
  {
    id: 'camera',
    label: 'Step 1 – Camera intrinsics (zed_camera_info.py)',
    keys: new Set<string>(),
    skipFlag: '--skip-camera-info',
  },
  {
    id: 'slam',
    label: 'Step 2 – SLAM (process_svo.py)',
    keys: new Set(['render', 'superpoint', 'quality', 'regen_grid']),
    includeRtabmap: true,
    skipFlag: '--skip-slam',
  },
  {
    id: 'video',
    label: 'Step 3 – SVO export (svo_export.py)',
    keys: new Set(['side', 'depth_compression']),
    skipFlag: null,
  },
  {
    id: 'poses',
    label: 'Step 4 – Pose conversion (convert_poses.py)',
    keys: new Set<string>(),
    skipFlag: '--skip-poses',
  },
  {
    id: 'projection',
    label: 'Step 5 – 2D projection (project_ply.py)',
    keys: new Set(['min_z', 'max_z', 'resolution']),
    skipFlag: '--skip-projection',
  },
  {
    id: 'zip',
    label: 'Step 6 – ZIP assembly',
    keys: new Set<string>(),
    skipFlag: '--skip-zip',
  },
]

function ParamField({
  p,
  value,
  onChange,
  disabled,
}: {
  p: ParamDef
  value: string
  onChange: (v: string) => void
  disabled: boolean
}) {
  if (p.type === 'bool') {
    return (
      <div className="flex items-center gap-2">
        <input
          type="checkbox"
          id={`param-${p.key}`}
          checked={value === 'true'}
          onChange={e => onChange(e.target.checked ? 'true' : 'false')}
          disabled={disabled}
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
        label={p.label}
        value={value}
        onChange={onChange}
        disabled={disabled}
        options={choices.map(c => ({ value: c, label: c }))}
      />
    )
  }
  return (
    <Input
      label={p.label}
      value={value}
      onChange={onChange}
      disabled={disabled}
      type={p.type === 'float' || p.type === 'int' ? 'number' : 'text'}
      placeholder={p.value}
    />
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
  const [outputName, setOutputName] = useState('')
  const [mapChoice, setMapChoice] = useState<'1' | '2'>('1')
  const [presetParams, setPresetParams] = useState<ParamDef[]>([])
  const [paramValues, setParamValues] = useState<Record<string, string>>({})
  const [showAdvanced, setShowAdvanced] = useState(false)
  const [stepEnabled, setStepEnabled] = useState<Record<string, boolean>>({
    camera: true, slam: true, video: true, poses: true, projection: true, zip: true,
  })
  const [openSteps, setOpenSteps] = useState<Record<string, boolean>>({
    camera: false, slam: false, video: false, poses: false, projection: false, zip: false,
  })
  const [exportVideo, setExportVideo] = useState(true)
  const [exportDepth, setExportDepth] = useState(true)
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
      if (data.startsWith('[DONE] exit=')) {
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
      if (data.startsWith('[DONE] exit=')) {
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
    if (!selectedConfig) { alert('Select a preset'); return }
    if (!selectedSvo) { alert('Select an SVO2 file'); return }
    if (STEP_GROUPS.every(g => !stepEnabled[g.id])) {
      toast(EASTER_EGG_MESSAGES[Math.floor(Math.random() * EASTER_EGG_MESSAGES.length)], {
        icon: '🤔',
        duration: 6000,
        style: { maxWidth: '480px' },
      })
      return
    }
    setLogs([])
    setZipReady(false)
    try {
      const extra_args: string[] = []
      // Step enable/disable flags
      for (const group of STEP_GROUPS) {
        if (!group.skipFlag) continue
        if (!stepEnabled[group.id]) {
          extra_args.push(group.skipFlag)
        }
      }
      // Video step: two independent sub-flags
      if (!stepEnabled['video']) {
        extra_args.push('--skip-video', '--skip-depth')
      } else {
        if (!exportVideo) extra_args.push('--skip-video')
        if (!exportDepth) extra_args.push('--skip-depth')
      }
      // Param values (run_pipeline.py / process_svo.py handle unused params gracefully)
      for (const p of presetParams) {
        const val = paramValues[p.key] ?? p.value
        if (p.type === 'bool') {
          if (val === 'true') {
            extra_args.push(p.cli)
          }
        } else {
          extra_args.push(p.cli, val)
        }
      }
      const payload: PipelinePayload = {
        config: selectedConfig,
        svo_stem: selectedSvo,
        output_name: outputName.trim() || selectedSvo,
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
  const zipSession = outputName.trim() || selectedSvo

  // ── Render ─────────────────────────────────────────────────────
  return (
    <div className="min-h-screen bg-gray-950 text-gray-100">
      <Toaster position="bottom-center" toastOptions={{ style: { background: '#1f2937', color: '#f3f4f6', border: '1px solid #374151' } }} />
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

          {/* Advanced settings – capture */}
          <div>
            <button
              onClick={() => setShowCaptureAdvanced(v => !v)}
              disabled={isRecording}
              className="flex items-center gap-1.5 text-sm text-gray-400 hover:text-gray-200 transition-colors disabled:opacity-40"
            >
              <span>{showCaptureAdvanced ? '▾' : '▸'}</span>
              <span>Advanced settings</span>
            </button>
            {showCaptureAdvanced && (
              <div className="mt-3 grid grid-cols-2 gap-3">
                <Input
                  label="imu_warmup (warmup, s)"
                  value={imuWarmup}
                  onChange={setImuWarmup}
                  disabled={isRecording}
                  type="number"
                  placeholder="2.0"
                />
                <Input
                  label="wait (start delay, s)"
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
                {frameCount > 0 ? `${frameCount.toLocaleString()} frames` : 'Initializing…'}
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
            <label className="block text-xs text-gray-400">SVO2 file</label>
            <select
              className="w-full bg-gray-800 border border-gray-700 rounded-lg px-3 py-2 text-sm
                         focus:outline-none focus:border-blue-500 disabled:opacity-40"
              value={selectedSvo}
              onFocus={fetchSvos}
              onChange={e => { setSelectedSvo(e.target.value); setOutputName(e.target.value) }}
              disabled={isPipelineRunning}
            >
              <option value="">— select —</option>
              {svos.map(s => <option key={s} value={s}>{s}.svo2</option>)}
            </select>
          </div>

          {/* Output folder name */}
          <Input
            label="Output folder (data/outputs/…)"
            value={outputName}
            onChange={setOutputName}
            placeholder={selectedSvo || 'output name'}
            disabled={isPipelineRunning}
          />

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

          {/* Advanced parameters — step accordions */}
          <div>
            <button
              onClick={() => setShowAdvanced(v => !v)}
              disabled={isPipelineRunning}
              className="flex items-center gap-1.5 text-sm text-gray-400 hover:text-gray-200 transition-colors disabled:opacity-40"
            >
              <span>{showAdvanced ? '▾' : '▸'}</span>
              <span>Advanced parameters</span>
            </button>

            {showAdvanced && (
              <div className="mt-3 space-y-3">

                {/* Global params (apply to multiple steps) */}
                {(() => {
                  const globals = presetParams.filter(p => GLOBAL_PARAM_KEYS.has(p.key))
                  if (globals.length === 0) return null
                  return (
                    <div className="grid grid-cols-2 gap-3 pb-1">
                      {globals.map(p => (
                        <ParamField
                          key={p.key}
                          p={p}
                          value={paramValues[p.key] ?? p.value}
                          onChange={v => setParamValues(prev => ({ ...prev, [p.key]: v }))}
                          disabled={isPipelineRunning}
                        />
                      ))}
                    </div>
                  )
                })()}

                {/* Per-step accordions — all 6, always rendered */}
                {STEP_GROUPS.map(group => {
                  const groupParams = presetParams.filter(p =>
                    group.keys.has(p.key) ||
                    (group.includeRtabmap === true && p.key.startsWith('rtabmap.'))
                  )
                  const hasContent = groupParams.length > 0 || group.id === 'zip' || group.id === 'video'
                  const isEnabled = stepEnabled[group.id] ?? true
                  const isOpen = openSteps[group.id] ?? false
                  return (
                    <div key={group.id} className="border border-gray-700 rounded-xl overflow-hidden">
                      {/* Header: checkbox + label + expand arrow */}
                      <div className="flex items-center gap-2 px-3 py-2.5 bg-gray-800/70">
                        <input
                          type="checkbox"
                          checked={isEnabled}
                          onChange={e => setStepEnabled(prev => ({ ...prev, [group.id]: e.target.checked }))}
                          disabled={isPipelineRunning}
                          className="h-4 w-4 flex-shrink-0 rounded border-gray-600 accent-blue-500 disabled:opacity-40"
                        />
                        <button
                          onClick={() => hasContent && setOpenSteps(prev => ({ ...prev, [group.id]: !prev[group.id] }))}
                          disabled={isPipelineRunning || !hasContent}
                          className="flex-1 flex items-center justify-between text-left gap-2 disabled:cursor-default"
                        >
                          <span className={`text-xs font-medium ${
                            isEnabled ? 'text-gray-200' : 'line-through text-gray-500'
                          }`}>
                            {group.label}
                          </span>
                          {hasContent && (
                            <span className="text-gray-500 text-xs flex-shrink-0">{isOpen ? '▾' : '▸'}</span>
                          )}
                        </button>
                      </div>

                      {/* Params body */}
                      {isOpen && hasContent && (
                        <div className="p-3 space-y-3 border-t border-gray-700">
                          {group.id === 'zip' && (
                            <Select
                              label="map_choice (map source for ZIP)"
                              value={mapChoice}
                              onChange={v => setMapChoice(v as '1' | '2')}
                              disabled={isPipelineRunning || !isEnabled}
                              options={[
                                { value: '1', label: 'RTAB-Map built-in (map.pgm)' },
                                { value: '2', label: 'Manual projection (map_manual.pgm)' },
                              ]}
                            />
                          )}
                          {group.id === 'video' ? (() => {
                            const sideParam = groupParams.find(p => p.key === 'side')
                            const comprParam = groupParams.find(p => p.key === 'depth_compression')
                            const dis = isPipelineRunning || !isEnabled
                            return (
                              <div className="grid grid-cols-2 gap-3">
                                {/* Col 1: side dropdown + Export MP4 checkbox */}
                                <div className="space-y-1">
                                  {sideParam && (
                                    <ParamField
                                      p={sideParam}
                                      value={paramValues[sideParam.key] ?? sideParam.value}
                                      onChange={v => setParamValues(prev => ({ ...prev, [sideParam.key]: v }))}
                                      disabled={dis || !exportVideo}
                                    />
                                  )}
                                  <div className="flex items-center gap-2 pt-1">
                                    <input
                                      type="checkbox"
                                      id="export-video-check"
                                      checked={exportVideo}
                                      onChange={e => setExportVideo(e.target.checked)}
                                      disabled={dis}
                                      className="h-4 w-4 rounded border-gray-600 bg-gray-800 accent-blue-500 disabled:opacity-40"
                                    />
                                    <label htmlFor="export-video-check" className="text-xs text-gray-300">Export MP4</label>
                                  </div>
                                </div>
                                {/* Col 2: depth compression + Export depth checkbox */}
                                <div className="space-y-1">
                                  {comprParam && (
                                    <ParamField
                                      p={comprParam}
                                      value={paramValues[comprParam.key] ?? comprParam.value}
                                      onChange={v => setParamValues(prev => ({ ...prev, [comprParam.key]: v }))}
                                      disabled={dis || !exportDepth}
                                    />
                                  )}
                                  <div className="flex items-center gap-2 pt-1">
                                    <input
                                      type="checkbox"
                                      id="export-depth-check"
                                      checked={exportDepth}
                                      onChange={e => setExportDepth(e.target.checked)}
                                      disabled={dis}
                                      className="h-4 w-4 rounded border-gray-600 bg-gray-800 accent-blue-500 disabled:opacity-40"
                                    />
                                    <label htmlFor="export-depth-check" className="text-xs text-gray-300">Export depth</label>
                                  </div>
                                </div>
                              </div>
                            )
                          })() : groupParams.length > 0 && (
                            <div className="grid grid-cols-2 gap-3">
                              {groupParams.map(p => (
                                <ParamField
                                  key={p.key}
                                  p={p}
                                  value={paramValues[p.key] ?? p.value}
                                  onChange={v => setParamValues(prev => ({ ...prev, [p.key]: v }))}
                                  disabled={isPipelineRunning || !isEnabled}
                                />
                              ))}
                            </div>
                          )}
                        </div>
                      )}
                    </div>
                  )
                })}

              </div>
            )}
          </div>

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
