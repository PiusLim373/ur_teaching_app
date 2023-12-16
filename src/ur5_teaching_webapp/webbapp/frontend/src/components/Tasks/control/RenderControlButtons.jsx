import React, { useState } from "react";
import { Button } from "react-bootstrap";

export default function RenderControlButtons(props) {
  const [controlList, setcontrolList] = useState([
    "sequence",
    "fallback",
    "succeeder",
    "failer",
    "retry",
  ]);
  return (
    <React.Fragment>
      {controlList.map((task, index) => (
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
