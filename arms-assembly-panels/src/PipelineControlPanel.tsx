import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useRef, useState } from "react";
import { createRoot } from "react-dom/client";

type StartPipelineResponse = {
  accepted: boolean;
  message: string;
};

type ListModelsResponse = {
  model_files: string[];
};

type StatusMessage = {
  data: string;
};

function basename(path: string): string {
  return path.split("/").pop() ?? path;
}

function PipelineControlPanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [models, setModels] = useState<string[]>([]);
  const [selectedModel, setSelectedModel] = useState("");
  const [generateGrasps, setGenerateGrasps] = useState(true);
  const [generateJigs, setGenerateJigs] = useState(true);
  const [collisionThreshold, setCollisionThreshold] = useState(0);
  const [running, setRunning] = useState(false);
  const [log, setLog] = useState<string[]>(["Waiting for assembler..."]);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const logEndRef = useRef<HTMLDivElement>(null);

  useLayoutEffect(() => {
    context.subscribe([{ topic: "/assembler/pipeline_status" }]);
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame as MessageEvent<StatusMessage>[]) {
          if (msg.topic === "/assembler/pipeline_status") {
            const text = msg.message.data;
            setLog((prev) => [...prev, text]);
            if (text === "Done." || text.startsWith("Error")) {
              setRunning(false);
            }
          }
        }
      }
    };
    context.watch("currentFrame");
  }, [context]);

  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  // Auto-scroll log to bottom
  useEffect(() => {
    logEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [log]);

  const refreshModels = useCallback(async () => {
    if (!context.callService) return;
    try {
      const response = (await context.callService(
        "/assembler/list_models", {}
      )) as ListModelsResponse;
      const files = response.model_files ?? [];
      setModels(files);
      if (files.length > 0 && !files.includes(selectedModel)) {
        setSelectedModel(files[0]!);
      }
    } catch (e) {
      setLog((prev) => [...prev, `Could not list models: ${String(e)}`]);
    }
  }, [context, selectedModel]);

  // Fetch model list once the bridge is available
  useEffect(() => {
    const timer = setTimeout(() => { void refreshModels(); }, 1500);
    return () => { clearTimeout(timer); };
  }, [refreshModels]);

  const handleRun = useCallback(async () => {
    if (!context.callService) {
      setLog(["Error: callService unavailable — is foxglove_bridge connected?"]);
      return;
    }
    if (!selectedModel) {
      setLog(["Error: no model selected"]);
      return;
    }

    setRunning(true);
    setLog([`Starting pipeline: ${basename(selectedModel)}`]);

    try {
      const response = (await context.callService("/assembler/start_pipeline", {
        model_file: selectedModel,
        generate_grasps: generateGrasps,
        generate_jigs: generateJigs,
        collision_volume_threshold: collisionThreshold,
      })) as StartPipelineResponse;

      if (!response.accepted) {
        setLog((prev) => [...prev, `Rejected: ${response.message}`]);
        setRunning(false);
      }
    } catch (e) {
      setLog((prev) => [...prev, `Error: ${String(e)}`]);
      setRunning(false);
    }
  }, [context, selectedModel, generateGrasps, generateJigs, collisionThreshold]);

  return (
    <div style={{ padding: 12, fontFamily: "sans-serif", userSelect: "none", display: "flex", flexDirection: "column", height: "100%" }}>
      <div style={{ fontWeight: "bold", fontSize: 14, marginBottom: 10 }}>Pipeline Control</div>

      {/* Model selector */}
      <label style={labelStyle}>Model</label>
      <div style={{ display: "flex", gap: 6, marginBottom: 10 }}>
        <select
          value={selectedModel}
          onChange={(e) => { setSelectedModel(e.target.value); }}
          disabled={running}
          style={{ ...inputStyle, flex: 1, marginBottom: 0 }}
        >
          {models.length === 0 && (
            <option value="">— no models found —</option>
          )}
          {models.map((f) => (
            <option key={f} value={f}>{basename(f)}</option>
          ))}
        </select>
        <button
          onClick={() => { void refreshModels(); }}
          disabled={running}
          title="Refresh model list"
          style={{ ...btnStyle, width: 32, padding: "4px 0", flexShrink: 0 }}
        >
          ↻
        </button>
      </div>

      {/* Options */}
      <div style={{ display: "flex", gap: 16, marginBottom: 10, flexWrap: "wrap" }}>
        <label style={checkStyle}>
          <input
            type="checkbox"
            checked={generateGrasps}
            disabled={running}
            onChange={(e) => { setGenerateGrasps(e.target.checked); }}
          />
          {" "}Generate grasps
        </label>
        <label style={checkStyle}>
          <input
            type="checkbox"
            checked={generateJigs}
            disabled={running}
            onChange={(e) => { setGenerateJigs(e.target.checked); }}
          />
          {" "}Generate jigs
        </label>
      </div>

      <label style={labelStyle}>Collision volume threshold (mm³)</label>
      <input
        type="number"
        min={0}
        value={collisionThreshold}
        onChange={(e) => { setCollisionThreshold(parseFloat(e.target.value) || 0); }}
        disabled={running}
        style={{ ...inputStyle, width: 120 }}
      />

      {/* Run button */}
      <button
        onClick={() => { void handleRun(); }}
        disabled={running || !selectedModel}
        style={{ ...btnStyle, marginTop: 12, marginBottom: 12, opacity: running || !selectedModel ? 0.5 : 1 }}
      >
        {running ? "Running…" : "▶  Run Pipeline"}
      </button>

      {/* Status log */}
      <div style={{ fontSize: 11, color: "#aaa", marginBottom: 4 }}>Status</div>
      <div style={{
        flex: 1,
        overflowY: "auto",
        background: "#111",
        borderRadius: 4,
        padding: "6px 8px",
        fontSize: 11,
        fontFamily: "monospace",
        color: "#ccc",
        minHeight: 80,
      }}>
        {log.map((line, i) => (
          <div key={i} style={{ color: line.startsWith("Error") || line.startsWith("Rejected") ? "#f88" : line === "Done." ? "#8f8" : "#ccc" }}>
            {line}
          </div>
        ))}
        <div ref={logEndRef} />
      </div>
    </div>
  );
}

const labelStyle: React.CSSProperties = {
  fontSize: 11,
  color: "#aaa",
  marginBottom: 3,
};

const inputStyle: React.CSSProperties = {
  width: "100%",
  marginBottom: 10,
  padding: "4px 8px",
  background: "#1e1e1e",
  border: "1px solid #444",
  borderRadius: 4,
  color: "#eee",
  fontSize: 12,
  boxSizing: "border-box",
};

const checkStyle: React.CSSProperties = {
  fontSize: 12,
  color: "#ccc",
  cursor: "pointer",
};

const btnStyle: React.CSSProperties = {
  padding: "6px 16px",
  cursor: "pointer",
  borderRadius: 4,
  border: "1px solid #555",
  background: "#1a4a7a",
  color: "#eee",
  fontSize: 13,
  width: "100%",
};

export function initPipelineControlPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<PipelineControlPanel context={context} />);
  return () => {
    root.unmount();
  };
}
