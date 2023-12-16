import React from "react";
import { useEffect, useState } from "react";
import { Card, Button, Row, Col, Table } from "react-bootstrap";
import { useNavigate } from "react-router-dom";
import axios from "axios";

// DnDTree
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

const ip = "localhost";

export default function Deploy(props) {
  const navigate = useNavigate();
  const [savedTasks, setSavedTasks] = useState([]);

  useEffect(() => {
    axios.get("http://" + ip + ":5000/api/teach/get_saved").then((res) => {
      if (res.data) {
        console.log(res.data.saved_tasks);
        setSavedTasks(res.data.saved_tasks);
      }
    });
  }, []);

  const activateTask = (event) => {
    axios
      .get(
        "http://" +
          ip +
          ":5000/api/mission_control/activate/" +
          event.target.value
      )
      .then((res) => {
        if (res.data.hasActivated) {
          navigate("/mission_control");
        } else {
          alert("There is an ongoing mission, please cancel and execute again");
        }
      });
  };
  const editTask = (event) => {
    axios
      .post("http://" + ip + ":5000/api/teach/", {
        edit_saved_id: event.target.value,
      })
      .then((res) => {
        if (res.data.success) {
          navigate("/");
        } else {
          alert("Error while setting task to editing mode");
        }
      });
  };
  const deleteTask = (event) => {
    axios
      .get(
        "http://" +
          ip +
          ":5000/api/mission_control/delete/" +
          event.target.value
      )
      .then((res) => {
        if (res.data.success) {
          alert("Task deleted successfully");
        } else {
          alert("There is an ongoing mission, please cancel and execute again");
        }
      });
  };

  return (
    <React.Fragment>
      <h1>Saved Tasks:</h1>
      <Button className="m-1" variant="secondary" onClick={editTask} value="">
        Create New Task
      </Button>
      <br />
      <Row>
        {savedTasks.map((task) => (
          <React.Fragment key={task._id}>
            <Col md={6}>
              <Card className="mt-4">
                <Card.Body>
                  <Card.Title>{task.name}</Card.Title>
                  {/* <ThemeProvider theme={theme}> */}
                  {/* <CssBaseline /> */}
                  <DndProvider
                    backend={MultiBackend}
                    options={getBackendOptions()}
                  >
                    <div className={styles.app}>
                      <Tree
                        tree={task.task}
                        rootId={0}
                        render={(node, { depth, isOpen, onToggle }) => (
                          <CustomNode
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
                  <Button
                    value={task._id}
                    className="m-1"
                    onClick={activateTask}
                    variant="secondary"
                  >
                    Deploy
                  </Button>
                  <Button
                    value={task._id}
                    className="m-1"
                    onClick={editTask}
                    variant="warning"
                  >
                    Edit
                  </Button>
                  <Button
                    value={task._id}
                    className="m-1"
                    onClick={deleteTask}
                    variant="danger"
                  >
                    Delete
                  </Button>
                </Card.Body>
              </Card>
            </Col>
          </React.Fragment>
        ))}
      </Row>
    </React.Fragment>
  );
}
