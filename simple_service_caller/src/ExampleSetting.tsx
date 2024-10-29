import {
  PanelExtensionContext,
  Topic,
  MessageEvent,
  SettingsTreeAction,
} from "@foxglove/extension";
import {produce} from "immer";
import { set } from "lodash";
import { useLayoutEffect, useEffect, useState, useCallback } from "react";
import ReactDOM from "react-dom";
import ReactJson from "react-json-view";

// This is the type of state we will use to render the panel and also
// persist to the layout.
type State = {
  data: {
    label: string;
    topic?: string;
    visible: boolean;
  };
};

function ExampleSetting({ context }: { context: PanelExtensionContext }): JSX.Element {
  const [topics, setTopics] = useState<readonly Topic[] | undefined>();
  const [messages, setMessages] = useState<readonly MessageEvent<unknown>[] | undefined>();

  const [renderDone, setRenderDone] = useState<(() => void) | undefined>();

  // Build our panel state from the context's initialState, filling in any possibly missing values.
  const [state, setState] = useState<State>(() => {
    const partialState = context.initialState as Partial<State>;
    return {
      data: {
        label: partialState.data?.label ?? "Data",
        topic: partialState.data?.topic ?? "/pose",
        visible: partialState.data?.visible ?? true,
      },
    };
  });

  // Respond to actions from the settings editor to update our state.
  const actionHandler = useCallback(
    (action: SettingsTreeAction) => {
      if (action.action === "update") {
        const { path, value } = action.payload;
        // We use a combination of immer and lodash to produce a new state object so react will
        // re-render our panel. Because our data node contains a label & and visibility property
        // this will handle editing the label and toggling the node visibility without any special
        // handling.
        setState(produce((draft) => set(draft, path, value)));

        // If the topic was changed update our subscriptions.
        // if (path[1] === "topic") {
        //   context.subscribe([{ topic: value as string }]);
        // }
      }
    },
    [context],
  );

  // Update the settings editor every time our state or the list of available topics changes.
  useEffect(() => {
    context.saveState(state);

    // const topicOptions = (topics ?? []).map((topic) => ({ value: topic.name, label: topic.name }));

    // We set up our settings tree to mirror the shape of our panel state so we
    // can use the paths to values from the settings tree to directly update our state.
    context.updatePanelSettingsEditor({
      actionHandler,
      nodes: {
        data: {
          // Our label comes from the label in our state and will update to reflect changes to the
          // value in state.
          label: state.data.label,
          // Setting this to true allows the user to edit the label of this node.
          renamable: true,
          // A non-undefined value here allows the user to toggle the visibility of this node.
          visible: state.data.visible,
          icon: "Cube",
          fields: {
            topic: {
              label: "Topic",
              input: "string",
              value: state.data.topic,
            },
          },
        },
      },
    });
  }, [context, actionHandler, state, topics]);

  // We use a layout effect to setup render handling for our panel. We also setup some topic
  // subscriptions.
  useLayoutEffect(() => {
    // The render handler is invoked by Foxglove during playback when your panel needs
    // to render because the fields it is watching have changed. How you handle rendering depends on
    // your framework. You can only setup one render handler - usually early on in setting up your
    // panel.  Without a render handler your panel will never receive updates.  The render handler
    // could be invoked as often as 60hz during playback if fields are changing often.
    context.onRender = (renderState, done) => {
      // render functions receive a _done_ callback. You MUST call this callback to indicate your
      // panel has finished rendering. Your panel will not receive another render callback until
      // _done_ is called from a prior render. If your panel is not done rendering before the next
      // render call, Foxglove shows a notification to the user that your panel is delayed.  Set the
      // done callback into a state variable to trigger a re-render.
      setRenderDone(() => done);

      // We may have new topics - since we are also watching for messages in the current frame,
      // topics may not have changed It is up to you to determine the correct action when state has
      // not changed.
      setTopics(renderState.topics);

      // currentFrame has messages on subscribed topics since the last render call
      if (renderState.currentFrame) {
        setMessages(renderState.currentFrame);
      }
    };

    // After adding a render handler, you must indicate which fields from RenderState will trigger
    // updates. If you do not watch any fields then your panel will never render since the panel
    // context will assume you do not want any updates.

    // tell the panel context that we care about any update to the _topic_ field of RenderState
    context.watch("topics");

    // tell the panel context we want messages for the current frame for topics we've subscribed to
    // This corresponds to the _currentFrame_ field of render state.
    context.watch("currentFrame");

    // Subscribe to our initial topic.
    if (state.data.topic) {
      context.subscribe([{ topic: state.data.topic }]);
    }
  }, [context]);

  // invoke the done callback once the render is complete
  useEffect(() => {
    renderDone?.();
  }, [renderDone]);

  return (
    <div style={{ padding: "1rem", display: "flex", flexDirection: "column", maxHeight: "100%" }}>
      <h2>{state.data.topic ?? "Choose a topic in settings"}</h2>
      <div
        style={{
          overflowY: "auto",
          flex: 1,
        }}
      >
        <ReactJson
          src={messages ?? {}}
        />
      </div>
    </div>
  );
}

export function initExampleSettingPanel(context: PanelExtensionContext): () => void {
  ReactDOM.render(<ExampleSetting context={context} />, context.panelElement);

  // Return a function to run when the panel is removed
  return () => {
    ReactDOM.unmountComponentAtNode(context.panelElement);
  };
}