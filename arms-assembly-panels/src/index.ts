import { ExtensionContext } from "@foxglove/extension";

import { initAssemblyStagePanel } from "./AssemblyStagePanel";
import { initPipelineControlPanel } from "./PipelineControlPanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "Assembly Stage Viewer",
    initPanel: initAssemblyStagePanel,
  });
  extensionContext.registerPanel({
    name: "Pipeline Control",
    initPanel: initPipelineControlPanel,
  });
}
