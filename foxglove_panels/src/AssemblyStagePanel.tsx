import React, { useCallback, useEffect, useState } from "react";
import { PanelExtensionContext } from "@foxglove/extension";

type StageResponse = {
  success: boolean;
  message: string;
};

function AssemblyStagePanel({ context }: { context: PanelExtensionContext }): React.ReactElement {
  const [stageIndex, setStageIndex] = useState(0);
  const [maxStage, setMaxStage] = useState(0);
  const [status, setStatus] = useState("Ready");
  const [calling, setCalling] = useState(false);

  const callSetStage = useCallback(
    async (index: number) => {
      if (!context.callService) {
        setStatus("callService not available — is foxglove_bridge running?");
        return;
      }
      setCalling(true);
      setStatus(`Setting stage ${index}...`);
      try {
        const response = (await context.callService("/assembler/set_stage", {
          stage_index: index,
        })) as StageResponse;

        if (!response.success) {
          // Parse max stage from error message "Stage index out of range (0–N)"
          const match = response.message.match(/0.(\d+)/);
          if (match) {
            const max = parseInt(match[1], 10);
            setMaxStage(max);
            setStatus(`Max stage updated: ${max}`);
          } else {
            setStatus(`Error: ${response.message}`);
          }
        } else {
          setStatus(`Showing stage ${index} of ${maxStage}`);
        }
      } catch (e) {
        setStatus(`Error: ${String(e)}`);
      }
      setCalling(false);
    },
    [context, maxStage]
  );

  // Probe stage 0 on mount to confirm the service is reachable
  useEffect(() => {
    callSetStage(0);
  }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const handleSlider = (e: React.ChangeEvent<HTMLInputElement>) => {
    const v = parseInt(e.target.value, 10);
    setStageIndex(v);
    callSetStage(v);
  };

  const step = (delta: number) => {
    const next = Math.min(Math.max(stageIndex + delta, 0), maxStage);
    setStageIndex(next);
    callSetStage(next);
  };

  return (
    <div style={{ padding: 12, fontFamily: "sans-serif", userSelect: "none" }}>
      <div style={{ marginBottom: 8, fontWeight: "bold", fontSize: 14 }}>
        Assembly Stage Viewer
      </div>

      <div style={{ display: "flex", alignItems: "center", gap: 8, marginBottom: 12 }}>
        <button onClick={() => step(-1)} disabled={calling || stageIndex <= 0} style={btnStyle}>
          ◀ Prev
        </button>

        <span style={{ minWidth: 70, textAlign: "center", fontSize: 13 }}>
          {stageIndex} / {maxStage}
        </span>

        <button onClick={() => step(1)} disabled={calling || stageIndex >= maxStage} style={btnStyle}>
          Next ▶
        </button>
      </div>

      <input
        type="range"
        min={0}
        max={maxStage}
        value={stageIndex}
        onChange={handleSlider}
        disabled={calling}
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

export default AssemblyStagePanel;
