import { PanelExtensionContext, RenderState, SettingsTreeAction} from "@foxglove/extension";
import { produce } from "immer";
import { set } from "lodash";
import { useCallback, useEffect, useLayoutEffect, useState } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";

type State = {
  request: string;
  response?: unknown;
  error?: Error | undefined;
  colorScheme?: RenderState["colorScheme"];
};

type SettingState = {
  service: {
    label: string;
    serviceName: string;
    fieldName?:string;
    visible: boolean;
  };
};

function SimpleServiceCallPanel({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();
  const [state, setState] = useState<State>({request: "Enter Text Here." });
  
  // Build our panel state from the context's initialState, filling in any possibly missing values.
  const [settingState, setSettingState] = useState<SettingState>(() => {
    const partialState = context.initialState as Partial<SettingState>;
    return {
      service: {
        label: partialState.service?.label ?? "Data",
        serviceName: partialState.service?.serviceName ?? "/",
        fieldName: partialState.service?.fieldName ?? "field name",
        visible: partialState.service?.visible ?? true,
      },
    };
  });

  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action === "update") {
        const { path, value } = action.payload;
        // We use a combination of immer and lodash to produce a new state object so react will
        // re-render our panel. Because our data node contains a label & and visibility property
        // this will handle editing the label and toggling the node visibility without any special
        // handling.
        
        setSettingState(produce((draft) => set(draft, path, value)));
        // setSettingState((previous) => {
        //   const newConfig = { ...previous };
        //   set(newConfig, path.slice(1), value);
        //   set(newConfig, path.slice(2), value);
        //   return newConfig;
        // })
        // // If the topic was changed update our subscriptions.
        // if (path[1] === "topic") {
        //   context.subscribe([{ topic: value as string }]);
        // }
      }
    },
    [context],
  );

  // Update the settings editor every time our state or the list of available topics changes.
  useEffect(() => {
    context.saveState(settingState);

    // We set up our settings tree to mirror the shape of our panel state so we
    // can use the paths to values from the settings tree to directly update our state.
    context.updatePanelSettingsEditor({
      actionHandler,
      nodes: {
        service: {
          // Our label comes from the label in our state and will update to reflect changes to the
          // value in state.
          label: settingState.service.label,
          // Setting this to true allows the user to edit the label of this node.
          renamable: true,
          fields: {
            serviceName: {
              label: "Service",
              input: "string",
              value: settingState.service.serviceName,
            },
            fieldName: {
              label: "FieldName",
              input: "string",
              value: settingState.service.fieldName,
            },

          },
        },
      },
    });
  }, [context, actionHandler, settingState, state]);

  // [context, actionHandler, settingState, state]);

  useLayoutEffect(() => {
    context.onRender = (renderState, done) => {
      setState((oldState) => ({ ...oldState, colorScheme: renderState.colorScheme }));
      setRenderDone(() => done)
    };
    

    context.watch("colorScheme");
    context.watch("currentFrame");

  }, [context, settingState.service.serviceName, settingState.service.fieldName]);


  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  

  const callService = useCallback(
    async (serviceName: string, request: string) => {
      if (!context.callService) {
        return;
      }

      try {
        const response = await context.callService(serviceName, JSON.parse(request));
        JSON.stringify(response); // Attempt serializing the response, to throw an error on failure
        setState((oldState) => ({
          ...oldState,
          response,
          error: undefined,
        }));
      } catch (error) {
        setState((oldState) => ({ ...oldState, error: error as Error }));
        console.error(error);
      }
    },
    [context.callService],
  );

  return (
    <div style={{ padding: "1rem" }}>
      {context.callService == undefined && (
        <p style={{ color: "red" }}>Calling services is not supported by this connection</p>
      )}
      
      <h4>Request</h4>
      <div>
        <textarea
          style={{ width: "100%", minHeight: "3rem" }}
          value={state.request}
          onChange={(event) => {
            setState({ ...state, request: event.target.value });
          }}
        />
      </div>
      <div>
        <button
          disabled={context.callService == undefined || settingState.service.serviceName === ""}
          style={{ width: "100%", minHeight: "2rem" }}
          // eslint-disable-next-line @typescript-eslint/no-misused-promises
          onClick={async () => {
            await callService(settingState.service.serviceName, `{"${settingState.service.fieldName}":"${state.request.replace(/(\r\n|\n|\r|\\|\t|⸓|")/gm,"")}"}`);
          }}
        >
          {`Call Service`}
        </button>
      </div>

      <div>
        <h4>Response</h4>
        <ReactJson
          name={null}
          src={state.error ? { error: state.error.message } : state.response ?? {}}
          indentWidth={2}
          enableClipboard={false}
          theme={state.colorScheme === "dark" ? "monokai" : "rjv-default"}
          displayDataTypes={false}
        />
      </div>
    </div>
  );
}

export function initSimpleServiceCallPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<SimpleServiceCallPanel context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}