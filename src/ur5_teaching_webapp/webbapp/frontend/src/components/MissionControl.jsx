import React from "react";
import { useEffect, useState } from "react";
import { Card, Button, Table } from "react-bootstrap";
import { Link } from "react-router-dom";
import { DndProvider } from "react-dnd";
import {
  Tree,
  MultiBackend,
  getBackendOptions,
} from "@minoru/react-dnd-treeview";
import { ThemeProvider, CssBaseline } from "@mui/material";
import { CustomNode } from "./DndUtils/CustomNode";
import { theme } from "./Tasks/theme";
import styles from "./DndUtils/Tree.module.css";

import axios from "axios";

const ip = "localhost";

export default function MissionControl() {
  const [hasFinishedInitialise, setHasFinishedInitialise] = useState(false);
  const [activeMissionID, setActiveMissionID] = useState(""); //empty means no activemission
  const [missionDetail, setMissionDetail] = useState({});
  const [intervalID, setIntervalID] = useState(0);

  var hasIntervalSpawned = false;

  useEffect(() => {
    axios.get("http://" + ip + ":5000/api/mission_control/").then((res) => {
      if (res.data) {
        setActiveMissionID(res.data.activeMissionID);
        if (res.data.activeMissionID !== "" && !hasIntervalSpawned) {
          let interval = setInterval(
            function () {
              poolMissionDetail(res.data.activeMissionID);
            },

            1000
          );
          setIntervalID(interval);
          hasIntervalSpawned = true;
          console.log(activeMissionID);
        } else if (res.data.activeMissionID === "")
          setHasFinishedInitialise(true);
      }
    });
    return () => {
      clearInterval(intervalID);
      hasIntervalSpawned = false;
    };
  }, []);

  const poolMissionDetail = (event) => {
    axios
      .get("http://" + ip + ":5000/api/mission_control/detail/" + event)
      .then((res) => {
        if (res.data) {
          console.log(res.data);
          setMissionDetail(res.data);
          if (!hasFinishedInitialise) setHasFinishedInitialise(true);
        }
      });
  };

  const terminateTask = () => {
    alert("Terminate task?");
    axios
      .get("http://" + ip + ":5000/api/mission_control/terminate/")
      .then((res) => {
        setActiveMissionID("");
        console.log(intervalID);
        clearInterval(intervalID);
        hasIntervalSpawned = false;
      });
  };

  return (
    <React.Fragment>
      {hasFinishedInitialise ? (
        <React.Fragment>
          <h1>Mission Control</h1>
          <br />
          {activeMissionID !== "" ? (
            <React.Fragment>
              <ThemeProvider theme={theme}>
                <CssBaseline />
                <DndProvider
                  backend={MultiBackend}
                  options={getBackendOptions()}
                >
                  <div className={styles.app}>
                    <Tree
                      tree={missionDetail.task}
                      rootId={0}
                      render={(node, { depth, isOpen, onToggle }) => (
                        <CustomNode
                          displayStatusColor={true}
                          node={node}
                          depth={depth}
                          isOpen={isOpen}
                        />
                      )}
                      sort={false}
                      insertDroppableFirst={false}
                      dropTargetOffset={5}
                      initialOpen={true}
                    />
                  </div>
                </DndProvider>
              </ThemeProvider>
              <Button
                variant="danger"
                style={{ width: "100%", height: "50px", marginBottom: "20px" }}
                onClick={() => terminateTask()}
              >
                Terminate task
              </Button>
            </React.Fragment>
          ) : (
            <h2>No active mission at the moment</h2>
          )}
        </React.Fragment>
      ) : (
        <React.Fragment>
          <h1>Loading...</h1>
        </React.Fragment>
      )}
    </React.Fragment>
  );
}
