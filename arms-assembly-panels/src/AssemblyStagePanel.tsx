import { PanelExtensionContext, MessageEvent } from "@foxglove/extension";
import { ReactElement, useCallback, useEffect, useLayoutEffect, useState } from "react";
import { createRoot } from "react-dom/client";

type StageResponse = {
  success: boolean;
  message: string;
  num_stages: number;
};

type StatusMessage = {
  data: string;
};

function AssemblyStagePanel({ context }: { context: PanelExtensionContext }): ReactElement {
  const [stageIndex, setStageIndex] = useState(0);
  const [maxStage, setMaxStage] = useState(0);
  const [status, setStatus] = useState("Waiting for pipeline...");
  const [calling, setCalling] = useState(false);
  const [ready, setReady] = useState(false);
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Subscribe to pipeline status so we auto-refresh when the pipeline finishes
  useLayoutEffect(() => {
    context.subscribe([{ topic: "/assembler/pipeline_status" }]);
    context.onRender = (renderState, done) => {
      setRenderDone(() => done);
      if (renderState.currentFrame) {
        for (const msg of renderState.currentFrame as MessageEvent<StatusMessage>[]) {
          if (msg.topic === "/assembler/pipeline_status") {
            const text = msg.message.data;
            if (text === "Resetting state...") {
              // New run starting — go back to waiting state
              setReady(false);
              setStageIndex(0);
              setMaxStage(0);
              setStatus("Waiting for pipeline...");
            } else if (text === "Done.") {
              // Pipeline just finished — probe stage 0 to get the new stage count
              void callSetStageRef.current(0);
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

  const callSetStage = useCallback(
    async (index: number) => {
      if (!context.callService) {
        setStatus("callService unavailable — is foxglove_bridge connected?");
        return;
      }
      setCalling(true);
      try {
        const response = (await context.callService("/assembler/set_stage", {
          stage_index: index,
        })) as StageResponse;

        if (response.num_stages > 0) {
          setMaxStage(response.num_stages - 1);
          setReady(true);
        }
        if (!response.success) {
          setStatus(`Error: ${response.message}`);
        } else {
          setStageIndex(index);
          setStatus(`Stage ${index} of ${response.num_stages - 1}`);
        }
      } catch (e) {
        setStatus(`Error: ${String(e)}`);
      }
      setCalling(false);
    },
    [context],
  );

  // Keep a ref so the onRender closure can call the latest version without stale capture
  const callSetStageRef = { current: callSetStage };

  const handleSlider = (e: React.ChangeEvent<HTMLInputElement>) => {
    const v = parseInt(e.target.value, 10);
    setStageIndex(v);
    void callSetStage(v);
  };

  const step = (delta: number) => {
    const next = Math.min(Math.max(stageIndex + delta, 0), maxStage);
    setStageIndex(next);
    void callSetStage(next);
  };

  return (
    <div style={{ padding: 12, fontFamily: "sans-serif", userSelect: "none" }}>
      <div style={{ marginBottom: 8, fontWeight: "bold", fontSize: 14 }}>
        Assembly Stage Viewer
      </div>

      {!ready && (
        <div style={{ fontSize: 12, color: "#888", marginBottom: 8 }}>
          Waiting for pipeline to complete…
        </div>
      )}

      <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 10 }}>
        <button
          onClick={() => { step(-1); }}
          disabled={calling || !ready || stageIndex <= 0}
          style={btnStyle}
        >
          ◀ Prev
        </button>

        <span style={{ minWidth: 70, textAlign: "center", fontSize: 13 }}>
          {ready ? `${stageIndex} / ${maxStage}` : "— / —"}
        </span>

        <button
          onClick={() => { step(1); }}
          disabled={calling || !ready || stageIndex >= maxStage}
          style={btnStyle}
        >
          Next ▶
        </button>
      </div>

      <input
        type="range"
        min={0}
        max={maxStage}
        value={stageIndex}
        onChange={handleSlider}
        disabled={calling || !ready}
        style={{ width: "100%", marginBottom: 8 }}
      />

      <div style={{ fontSize: 11, color: "#888" }}>{status}</div>
    </div>
  );
}

const btnStyle: React.CSSProperties = {
  padding: "4px 12px",
  cursor: "pointer",
  borderRadius: 4,
  border: "1px solid #555",
  background: "#2a2a2a",
  color: "#eee",
};

export function initAssemblyStagePanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(<AssemblyStagePanel context={context} />);
  return () => {
    root.unmount();
  };
}
