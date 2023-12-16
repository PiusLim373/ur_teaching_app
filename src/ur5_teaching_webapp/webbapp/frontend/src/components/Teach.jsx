import React, { useState, useEffect, useCallback } from "react";
import { DndProvider } from "react-dnd";
import {
  Tree,
  MultiBackend,
  getBackendOptions,
} from "@minoru/react-dnd-treeview";
import SampleTree from "./Tasks/dnd_sample_data.json";
import EmptyTemplate from "./Tasks/dnd_empty_data.json";
import { CustomNode } from "./DndUtils/CustomNode";
import styles from "./DndUtils/Tree.module.css";
import { CustomDragPreview } from "./DndUtils/CustomDragPreview";
import { Placeholder } from "./DndUtils/Placeholder";
import EditIcon from "@mui/icons-material/Edit";
import {
  Row,
  Col,
  Button,
  Card,
  InputGroup,
  Form,
  Accordion,
  OverlayTrigger,
  Popover,
} from "react-bootstrap";
import globalVars from "../globalVars";
import axios from "axios";

import RenderMotionButtons from "./Tasks/motion_service/RenderMotionButtons";
import DnDSavedJoint from "./Tasks/motion_service/SavedJoint";
import DnDCustomJoint from "./Tasks/motion_service/CustomJoint";
import DnDSavedCartesian from "./Tasks/motion_service/SavedCartesian";
import DnDCustomCartesian from "./Tasks/motion_service/CustomCartesian";

import RenderControlButtons from "./Tasks/control/RenderControlButtons";
import Control from "./Tasks/control/Control";
import ControlWithInput from "./Tasks/control/ControlWithInput";

function App(props) {
  const [treeData, setTreeData] = useState(EmptyTemplate);
  const [taskIDInDatabase, setTaskIDInDatabse] = useState(""); // if taskIDInDatabase is not empty, means editing an existing task
  const [hasFinishedInitialise, setHasFinishedInitialise] = useState(true);
  const [selected, setSelected] = useState({}); // to not allow multiple selection
  const [nextID, setNextID] = useState(-1);
  const [showEdit, setShowEdit] = useState(false);
  const [tempNodeName, setTempNodeName] = useState("");
  const [taskName, setTaskName] = useState("");
  const flowNode = ["sequence", "fallback", "start"];
  const functionalNode = ["succeeder", "failer", "retry", "wait"];
  const handleDrop = (newTree) => setTreeData(newTree);

  const debug = (e) => {
    console.log(treeData);
    console.log(selected);
  };

  const markErrorNode = (id, valid, msg) => {
    const newTree = treeData.map((node, index) => {
      if (node.id === id) {
        let tempData = node.data;
        tempData.valid = valid;

        if (!valid) tempData.invalidMsg = msg;
        else tempData.invalidMsg = "";
        return {
          ...node,
          data: tempData,
        };
      }
      return node;
    });
    setTreeData(newTree);
  };

  const verifyTree = () => {
    //list out all control nodes to check
    let isTreeValid = true;
    let childObj = {};
    let flowNodeArr = [];
    let functionalNodeArr = [];
    let taskNodeArr = [];
    for (const node of treeData) {
      if (node.parent in childObj) childObj[node.parent].push(node.id);
      else childObj[node.parent] = [node.id];
      if (flowNode.includes(node.data.fileType)) flowNodeArr.push(node.id);
      else if (functionalNode.includes(node.data.fileType))
        functionalNodeArr.push(node.id);
      else taskNodeArr.push(node.id);
    }
    console.log(childObj);
    console.log(flowNodeArr);
    console.log(taskNodeArr);
    console.log(functionalNodeArr);

    // set all nodes to true first
    for (const node of treeData) {
      markErrorNode(node.id, true, "");
    }

    // check all flownodes
    for (const flowNodeToCheck of flowNodeArr) {
      console.log(flowNodeToCheck);
      if (!(flowNodeToCheck in childObj)) {
        console.log("not all control has child");
        markErrorNode(
          flowNodeToCheck,
          false,
          "Flow node must have at least one child"
        );
        isTreeValid = false;
      }
    }

    // check all functional nodes
    for (const functionalNodeToCheck of functionalNodeArr) {
      console.log(functionalNodeToCheck);
      if (!(functionalNodeToCheck in childObj)) {
        console.log("not all control has child");
        markErrorNode(
          functionalNodeToCheck,
          false,
          "Functional node must have a child"
        );
        isTreeValid = false;
      } else if (childObj[functionalNodeToCheck].length > 1) {
        console.log("Functional node can only have 1 child");
        markErrorNode(
          functionalNodeToCheck,
          false,
          "Functional node can only have 1 child"
        );
        isTreeValid = false;
      }
    }

    // check for multiple rootnode
    for (const node of treeData) {
      if (node.id !== 1 && node.parent === 0) {
        console.log("root node must be start node" + node.id);
        markErrorNode(node.id, false, "Root node can only consist of 'Start'");
        isTreeValid = false;
      }

      //check all task node
      if (taskNodeArr.includes(node.id)) {
        if (node.parent === 1) {
          markErrorNode(
            node.id,
            false,
            "Task node must be inside a control node"
          );
          isTreeValid = false;
        }
        if (!node.data.payloadDataValid) {
          isTreeValid = false;
        }
      }
    }
    console.log("isTreeValid verdict: " + isTreeValid);
    return isTreeValid;
  };

  useEffect(() => {
    axios.get("http://" + globalVars.ip + ":5000/api/teach/").then((res) => {
      if (res.data) {
        if (Object.keys(res.data.task_to_edit).length !== 0) {
          setTreeData(res.data.task_to_edit.task);
          setTaskName(res.data.task_to_edit.name);
          setTaskIDInDatabse(res.data.task_to_edit._id);
        }
        setHasFinishedInitialise(true);
      }
    });
  }, []);

  useEffect(() => {
    // update the selected obj whenever tree is changed
    if (Object.keys(selected).length !== 0) {
      console.log("tree updated, updating selected as well");
      let tempIndex = -1;
      treeData.map((node, index) => {
        if (node.id === selected.id) {
          tempIndex = index;
        }
      });
      setSelected(treeData[tempIndex]);
    }

    //update the nextID
    let maxID = -1;
    treeData.map((node) => {
      if (node.id > maxID) {
        setNextID(node.id + 1);
        maxID = node.id;
      }
    });
  }, [treeData]);

  const snackToCamel = (s) => {
    return s.replace(/([-_][a-z])/gi, ($1) => {
      return $1.toUpperCase().replace("-", "").replace("_", "");
    });
  };

  const addNode = (event) => {
    // creating a placeholder node
    console.log(event.target.value);
    let newNode = {
      id: nextID,
      parent: 1,
      droppable: true,
      text: snackToCamel(event.target.value) + "_" + nextID,
      data: {
        fileType: event.target.value,
        status: "",
        selected: false,
        ros_payload: {},
        valid: false,
        payloadDataValid: false,
        invalidMsg: "",
      },
    };
    switch (event.target.value) {
      case "sequence":
      case "fallback":
      case "succeeder":
      case "failer":
        newNode.data.ros_payload.service = "";
        newNode.data.ros_payload.task = "control_node";
        newNode.data.ros_payload.data = event.target.value;
        newNode.data.ros_payload.control_node_data = 0;
        newNode.data.ros_payload.children = [];
        newNode.droppable = true;
        newNode.data.payloadDataValid = true;
        newNode.data.invalidMsg = "Control node must have at least one child";
        break;
      case "retry":
        newNode.data.ros_payload.service = "";
        newNode.data.ros_payload.task = "control_node";
        newNode.data.ros_payload.data = event.target.value;
        newNode.data.ros_payload.control_node_data = 3;
        newNode.data.ros_payload.children = [];
        newNode.droppable = true;
        newNode.data.payloadDataValid = true;
        newNode.data.invalidMsg = "Control node must have at least one child";
        break;
      case "wait":
        newNode.data.ros_payload.service = "";
        newNode.data.ros_payload.task = "control_node";
        newNode.data.ros_payload.data = event.target.value;
        newNode.data.ros_payload.control_node_data = 0;
        newNode.data.ros_payload.children = [];
        newNode.droppable = true;
        newNode.data.payloadDataValid = true;
        newNode.data.invalidMsg = "Control node must have at least one child";
        break;
      case "saved_joint":
      case "saved_cartesian":
        newNode.data.ros_payload.service = "motion_service";
        newNode.data.ros_payload.task = event.target.value;
        newNode.data.ros_payload.data = "";
        newNode.data.ros_payload.children = [];
        newNode.droppable = false;
        newNode.data.invalidMsg = "Invalid payload data";
        break;
      case "custom_joint":
      case "custom_cartesian":
      case "jog":
        newNode.data.ros_payload.service = "motion_service";
        newNode.data.ros_payload.task = event.target.value;
        newNode.data.ros_payload.data = [];
        newNode.data.ros_payload.children = [];
        newNode.droppable = false;
        newNode.data.invalidMsg = "Invalid payload data";
        break;
    }

    if (Object.keys(selected).length === 0) newNode.parent = 1;
    else if (selected.droppable) newNode.parent = selected.id;
    else newNode.parent = selected.parent;

    console.log(newNode);

    // appending the node to treeData
    let insertPos = treeData.length - 1;
    if (Object.keys(selected).length !== 0) {
      insertPos = treeData.findIndex((object) => {
        return object.id === selected.id;
      });
      if (insertPos === -1) {
        console.log("something wrong during insertPos searching");
        insertPos = treeData.length - 1;
      }
    }
    const newTree = [...treeData];
    newTree.splice(insertPos + 1, 0, newNode);
    newTree[insertPos].data.selected = false;
    newTree[insertPos + 1].data.selected = true;
    setTreeData(newTree);

    // select the newest created node
    setSelected(newTree[insertPos + 1]);
    // increament the id for next creation
    setNextID(nextID + 1);
  };

  const handleSelect = (e) => {
    setSelected({});
    setShowEdit(false);
    if (Object.keys(selected).length !== 0) {
      const newTree = treeData.map((node) => {
        if (node.id === selected.id) {
          let tempData = node.data;
          tempData.selected = false;
          return {
            ...node,
            data: tempData,
          };
        }
        return node;
      });
      setTreeData(newTree);
      if (selected.id === e.id) {
        setSelected({});
        return;
      }
    }
    let selectedIndex = -1;
    const newTree = treeData.map((node, index) => {
      if (node.id === e.id) {
        let tempData = node.data;
        tempData.selected = !tempData.selected;
        selectedIndex = index;
        return {
          ...node,
          data: tempData,
        };
      }
      return node;
    });
    setSelected(treeData[selectedIndex]);
  };

  const handleShowEdit = () => {
    setShowEdit(true);
    setTempNodeName(selected.text);
  };

  const handleChangeNodeName = (e) => {
    setTempNodeName(e.target.value);
  };

  const handleDelete = (id) => {
    let index = treeData.map((node) => node.id).indexOf(selected.id);
    if (index !== -1) {
      // dont allow delete if there's child
      let childObj = {};
      for (const node of treeData) {
        if (node.parent in childObj) childObj[node.parent].push(node.id);
        else childObj[node.parent] = [node.id];
      }
      console.log(childObj);
      if (
        treeData[index].droppable &&
        treeData[index].id in childObj &&
        childObj[treeData[index].id].length > 0
      ) {
        alert("Please remove child component first")
      } else {
        setShowEdit(false);
        setSelected({});
        setTreeData([
          ...treeData.slice(0, index),
          ...treeData.slice(index + 1, treeData.length),
        ]);
      }
    }
  };

  const updateNodeName = () => {
    let tempIndex = -1;
    const newTree = treeData.map((node, index) => {
      if (node.id === selected.id) {
        tempIndex = index;
        return {
          ...node,
          text: tempNodeName,
        };
      }
      return node;
    });
    console.log(tempIndex);
    setTreeData(newTree);
    setTempNodeName("");
    setShowEdit(false);
  };

  const renderPayload = (type) => {
    switch (type) {
      case "sequence":
      case "fallback":
      case "succeeder":
      case "failer":
        return (
          <Control
            key={selected.id}
            selected={selected}
            teachDelete={handleDelete}
          ></Control>
        );
        break;
      case "retry":
      case "wait":
        return (
          <ControlWithInput
            key={selected.id}
            selected={selected}
            teachChange={onNodePayloadUpdate}
            teachDelete={handleDelete}
          ></ControlWithInput>
        );
        break;
      case "saved_joint":
        return (
          <DnDSavedJoint
            key={selected.id}
            selected={selected}
            teachChange={onNodePayloadUpdate}
            teachDelete={handleDelete}
          />
        );
        break;
      case "custom_joint":
        return (
          <DnDCustomJoint
            key={selected.id}
            selected={selected}
            teachChange={onNodePayloadUpdate}
            teachDelete={handleDelete}
          />
        );
        break;
      case "saved_cartesian":
        return (
          <DnDSavedCartesian
            key={selected.id}
            selected={selected}
            teachChange={onNodePayloadUpdate}
            teachDelete={handleDelete}
          />
        );
        break;
      case "custom_cartesian":
        return (
          <DnDCustomCartesian
            key={selected.id}
            selected={selected}
            teachChange={onNodePayloadUpdate}
            teachDelete={handleDelete}
          />
        );
        break;
      default:
        return <h1>Not set</h1>;
    }
  };

  const onNodePayloadUpdate = (newData) => {
    const newTree = treeData.map((node, index) => {
      if (node.id === newData.id) {
        return {
          ...node,
          data: newData.data,
        };
      }
      return node;
    });
    setTreeData(newTree);
  };

  const handleChangeTaskName = (e) => {
    setTaskName(e.target.value);
  };

  const consolidateAndSave = (e) => {
    let isTreeValid = verifyTree();

    //timeout function so that the background can be rendered before showing confirm window
    setTimeout(function () {
      // clear out selected node, else will render yellow
      if (Object.keys(selected).length !== 0) {
        const newTree = treeData.map((node) => {
          if (node.id === selected.id) {
            let tempData = node.data;
            tempData.selected = false;
            return {
              ...node,
              data: tempData,
            };
          }
          return node;
        });
        setTreeData(newTree);
        setSelected({});
      }

      // saving procedure
      if (isTreeValid && window.confirm("Confirm save?")) {
        if (e.target.value === "save") {
          // save to existing task
          axios
            .post(`http://${globalVars.ip}:5000/api/teach/save`, {
              taskID: taskIDInDatabase,
              task: treeData,
              name: taskName,
            })
            .then((res) => {
              if (res.data) {
                console.log(res);
                alert("Task saved, please to go Deploy page to activate it");
              }
            });
        } else {
          // save as a new task
          axios
            .post(`http://${globalVars.ip}:5000/api/teach/save`, {
              taskID: "",
              task: treeData,
              name: taskName,
            })
            .then((res) => {
              if (res.data) {
                console.log(res);
                alert("Task saved, please to go Deploy page to activate it");
              }
            });
        }
      } else if (!isTreeValid) {
        alert("Some component(s) of task is(are) invalid, please check");
      }
    }, 1);
  };

  return (
    <React.Fragment>
      <h1>Teaching:</h1>
      {hasFinishedInitialise ? (
        <React.Fragment>
          <Row>
            <Col md={6}>
              <InputGroup>
                <InputGroup.Text id="basic-addon1">Task Name</InputGroup.Text>
                <Form.Control
                  placeholder="Give this motion flow a name"
                  value={taskName}
                  onChange={handleChangeTaskName}
                />
              </InputGroup>
              <br />
              <DndProvider backend={MultiBackend} options={getBackendOptions()}>
                <div className={styles.app}>
                  <Tree
                    tree={treeData}
                    rootId={0}
                    render={(node, { depth, isOpen, onToggle }) => (
                      <CustomNode
                        node={node}
                        depth={depth}
                        isOpen={isOpen}
                        onToggle={onToggle}
                        onSelect={handleSelect}
                        selected={node.data.selected}
                      />
                    )}
                    dragPreviewRender={(monitorProps) => (
                      <CustomDragPreview monitorProps={monitorProps} />
                    )}
                    onDrop={handleDrop}
                    classes={{
                      root: styles.treeRoot,
                      draggingSource: styles.draggingSource,
                      placeholder: styles.placeholderContainer,
                    }}
                    sort={false}
                    insertDroppableFirst={false}
                    canDrop={(
                      tree,
                      { dragSource, dropTargetId, dropTarget }
                    ) => {
                      if (dragSource?.parent === dropTargetId) {
                        return true;
                      }
                    }}
                    dropTargetOffset={5}
                    placeholderRender={(node, { depth }) => (
                      <Placeholder node={node} depth={depth} />
                    )}
                    initialOpen={true}
                  />
                </div>
              </DndProvider>
            </Col>
            <Col md={6}>
              <Accordion defaultActiveKey="0">
                <Accordion.Item eventKey="0">
                  <Accordion.Header>Control Nodes</Accordion.Header>
                  <Accordion.Body>
                    <RenderControlButtons addNode={addNode} />
                  </Accordion.Body>
                </Accordion.Item>
                <Accordion.Item eventKey="1">
                  <Accordion.Header>Motion Service Nodes</Accordion.Header>
                  <Accordion.Body>
                    <RenderMotionButtons addNode={addNode} />
                  </Accordion.Body>
                </Accordion.Item>
              </Accordion>

              <br />
              <Card style={{ marginTop: "20px" }}>
                <Card.Body>
                  <Card.Title>
                    {showEdit ? (
                      <React.Fragment>
                        <InputGroup className="mb-3">
                          <Form.Control
                            placeholder="Edit name"
                            value={tempNodeName}
                            onChange={handleChangeNodeName}
                            isInvalid={tempNodeName === ""}
                          />
                          <Button
                            variant="outline-secondary"
                            disabled={tempNodeName === ""}
                            onClick={updateNodeName}
                          >
                            Save
                          </Button>
                        </InputGroup>
                      </React.Fragment>
                    ) : (
                      <React.Fragment>
                        {selected.text} <EditIcon onClick={handleShowEdit} />
                      </React.Fragment>
                    )}
                  </Card.Title>
                  <Card.Text>{selected.description}</Card.Text>
                  {Object.keys(selected).length !== 0
                    ? renderPayload(selected.data.fileType)
                    : "Select something to start"}
                </Card.Body>
              </Card>
            </Col>
          </Row>
          <br />
          {taskIDInDatabase !== "" ? (
            <h5 style={{ color: "red" }}>
              You are editing an existing task, you can choose to Save this as a
              new task below
            </h5>
          ) : (
            ""
          )}
          <Button onClick={consolidateAndSave} value="save" variant="success">
            Verify and Save
          </Button>
          {taskIDInDatabase !== "" ? (
            <Button
              onClick={consolidateAndSave}
              className="m-2"
              value="saveAs"
              variant="warning"
            >
              Verify and Save As
            </Button>
          ) : (
            ""
          )}
          <Button onClick={debug}>Debug</Button>
        </React.Fragment>
      ) : (
        <h2>Loading...</h2>
      )}
    </React.Fragment>
  );
}

export default App;
