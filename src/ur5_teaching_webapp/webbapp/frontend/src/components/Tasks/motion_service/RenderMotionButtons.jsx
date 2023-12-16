import React, { useState } from "react";
import { Button } from "react-bootstrap";

export default function RenderMotionButtons(props) {
  const [motionTaskList, setMotionTaskList] = useState([
    "saved_joint",
    "custom_joint",
    "saved_cartesian",
    "custom_cartesian",
    "jog",
  ]);
  return (
    <React.Fragment>
      {motionTaskList.map((task, index) => (
        <Button
          className="m-1"
          variant="secondary"
          key={index}
          value={task}
          onClick={props.addNode}
        >
          {task}
        </Button>
      ))}
    </React.Fragment>
  );
}
