import { useState, useEffect } from "react";
import { Button } from "@/components/ui/button";
import { Input } from "@/components/ui/input";
import { Tabs, TabsList, TabsTrigger, TabsContent } from "@/components/ui/tabs";
import { Panel } from "@/components/ui/panel";
import { Joystick } from "@/components/ui/joystick";
import minecraft from "../public/css/minecraft.css?raw";
import { cn } from "./lib/utils";
import PointCloud from "./components/world/PointCloud";
import {
  RosConnection,
  // ImageViewer,
  // Subscriber,
  TopicListProvider,
  // useMsg,
  useTopicList,
  // Publisher,
  // Param,
  // useParam,
  ParamListProvider,
  useParamList,
  ServiceListProvider,
  useServiceList,
  // ServiceCaller,
  // ServiceServer,
} from "./lib/rosreact/src";

const THEME = "default"; // "minecraft" | "default"

function App() {
  const [x, setX] = useState("");
  const [y, setY] = useState("");
  const [logs, setLogs] = useState<string[]>([]);
  const log = (text: string) => setLogs((prev) => [...prev, text]);
  const isMinecraft = THEME === "minecraft";

  // rosreact
  const [trigger, setTrigger] = useState(false);
  // const [delParam, setDelParam] = useState(false);
  // const [message, setMessage] = useState({ data: 0 });

  useEffect(() => {
    setTimeout(() => {
      setTrigger(!trigger);
    }, 3000);
  }, [trigger]);

  // useEffect(() => {
  //   setTimeout(() => {
  //     setMessage({ data: 4 });
  //   }, 3000);
  // }, []);

  // useEffect(() => {
  //   setTimeout(() => {
  //     setDelParam(true);
  //   }, 10000);
  // }, []);

  return (
    <RosConnection url={"ws://127.0.0.1:9090"} autoConnect>
      {isMinecraft && <style>{minecraft}</style>}
      <div
        className={cn(
          "min-h-screen p-6 font-mono [--block-size:48px]",
          isMinecraft && "text-white",
        )}
        style={
          isMinecraft
            ? {
                backgroundImage: "url('/imgs/items/apple-golden.png')",
                backgroundRepeat: "repeat",
                backgroundSize: "48px 48px",
              }
            : undefined
        }
      >
        <h1 className="text-shadow-xl mb-6 text-center text-2xl">TurboSLAM </h1>

        <Tabs defaultValue="user" className="w-full">
          <TabsList className="mb-4" variant={THEME}>
            <TabsTrigger value="user" variant={THEME}>
              User Mode
            </TabsTrigger>
            <TabsTrigger value="debug" variant={THEME}>
              Debug Mode
            </TabsTrigger>
          </TabsList>

          <TabsContent value="user">
            <div className="mb-4 flex flex-wrap gap-2">
              <Button onClick={() => log("Set Origin")} variant={THEME}>
                Set Origin
              </Button>
              <Button onClick={() => log("Exploring...")} variant={THEME}>
                Explore
              </Button>
              <Input
                placeholder="X"
                value={x}
                onChange={(e) => setX(e.target.value)}
                className="w-24"
                variant={THEME}
              />
              <Input
                placeholder="Y"
                value={y}
                onChange={(e) => setY(e.target.value)}
                className="w-24"
                variant={THEME}
              />
              <Button onClick={() => log(`GO to (${x}, ${y})`)} variant={THEME}>
                GO
              </Button>
            </div>

            <div className="flex gap-4">
              <Panel
                title="GPS Navigation"
                className="min-h-[var(--9-block)] flex-1"
              >
                <p>
                  Target: ({x}, {y})
                </p>

                <div className="map relative mt-2 mr-auto ml-auto h-[var(--7-block)] w-[95%] overflow-hidden rounded [--zoom:2]">
                  <div className="absolute right-2 bottom-2 scale-50">
                    <Joystick
                      onChange={(val) => console.log("Joystick moved:", val)}
                    />
                  </div>
                </div>
              </Panel>

              <Panel title="3D Map" className="min-h-[var(--6-block)] flex-1">
                <p>[Map display placeholder]</p>
              </Panel>
            </div>
          </TabsContent>

          <TabsContent value="debug">
            <Panel title="Debug Logs" variant="dark">
              <pre className="h-[var(--4-block)] overflow-y-scroll text-sm text-green-400">
                {logs.join("\n")}
              </pre>
            </Panel>

            {/* <Subscriber topic="/number" messageType="std_msgs/Float32">
                  <MsgView />
                </Subscriber> */}
            {/* <Param
                  name="/react/param"
                  setValue={1}
                  get={trigger}
                  delete={delParam}
                  deleteCallback={(resp) => {
                    console.log(resp);
                  }}
                  setCallback={(resp) => {
                    console.log(resp);
                  }}
                >
                  <ParamView />
                </Param> */}
            {/* <Publisher
                  autoRepeat
                  topic="/react/pub/repeat"
                  throttleRate={10.0}
                  message={{ data: 2 }}
                  messageType="std_msgs/Float32"
                />

                <Publisher
                  topic="/react/pub/norepeat"
                  throttleRate={10.0}
                  message={message}
                  messageType="std_msgs/Float32"
                  latch={true}
                /> */}
            {/* <ServiceServer
                  name="/react/service"
                  serviceType="std_srvs/SetBool"
                  callback={serviceServerCallback}
                />

                <ServiceCaller
                  name="/setbool"
                  serviceType="std_srvs/SetBool"
                  request={{ data: true }}
                  trigger={trigger}
                  callback={(resp) => {
                    console.log(resp);
                  }}
                  failedCallback={(error) => {
                    console.log(error);
                  }}
                /> */}
            <TopicListProvider
              trigger={trigger}
              failedCallback={(e) => {
                console.log(e);
              }}
            >
              <TopicListView />
            </TopicListProvider>
            <ServiceListProvider
              trigger={trigger}
              failedCallback={(e) => {
                console.log(e);
              }}
            >
              <ServiceListView />
            </ServiceListProvider>
            <ParamListProvider
              trigger={trigger}
              failedCallback={(e) => {
                console.log(e);
              }}
            >
              <ParamListView />
            </ParamListProvider>

            {/* TODO: wrap PointCloud in a subscriber that listens to LiDAR data*/}
            <PointCloud />
          </TabsContent>
        </Tabs>
      </div>
    </RosConnection>
  );
}

// const serviceServerCallback = (request: any, response: any) => {
//   if (request.data === true) {
//     response.success = true;
//     response.message = "Passed true value";
//   } else {
//     response.success = false;
//     response.message = "Passed false value";
//   }
// };

// const ParamView = () => {
//   const param = useParam();
//   return (
//     <>
//       <h2>Param</h2>
//       <p>{`${param}`}</p>
//     </>
//   );
// };

// const MsgView = () => {
//   const msg = useMsg();

//   return (
//     <>
//       <h2>Msg</h2>
//       <p> {`${msg}`} </p>
//     </>
//   );
// };

const TopicListView = () => {
  const topicList = useTopicList();
  return (
    <>
      <h2>Topics</h2>
      <p>{`${topicList.topics}`}</p>
      <p>{`${topicList.types}`}</p>
    </>
  );
};

const ServiceListView = () => {
  const list = useServiceList();
  return (
    <>
      <h2>Services</h2>
      <p>{`${list}`}</p>
    </>
  );
};

const ParamListView = () => {
  const list = useParamList();
  return (
    <>
      <h2>Parameters</h2>
      <p>{`${list}`}</p>
    </>
  );
};

export default App;
