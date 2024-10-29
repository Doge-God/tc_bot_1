import { ExtensionContext } from "@foxglove/extension";
// import { initExampleSettingPanel } from "./ExampleSetting";
// import { initExamplePanel } from "./ExamplePanel";
// import {initCallServicePanel} from "./ExampleCaller"
import { initSimpleServiceCallPanel } from "./SimpleServicePanel";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({ name: "SimpleServiceCall", initPanel: initSimpleServiceCallPanel});
}
