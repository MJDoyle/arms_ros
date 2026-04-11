import { ExtensionContext, PanelExtensionContext } from "@foxglove/extension";
import { createElement } from "react";
import { createRoot } from "react-dom/client";
import AssemblyStagePanel from "./AssemblyStagePanel";

function initPanel(context: PanelExtensionContext): () => void {
  const root = createRoot(context.panelElement);
  root.render(createElement(AssemblyStagePanel, { context }));
  return () => root.unmount();
}

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Assembly Stage Viewer",
    initPanel,
  });
}
