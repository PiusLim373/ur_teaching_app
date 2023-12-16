import React, { useState, useEffect } from "react";
import { Button, InputGroup, Form } from "react-bootstrap";

const controlNodeDescription = {
  sequence:
    "Execute node continuously, return True if all childnode returns True, stop execution and return False once a False childnode is encountered",
  fallback:
    "Execute node continuously, return False if all childnode returns False, stop execution and return True once a True childnode is encountered",
  succeeder: "Convert the childnode's return to True",
  failer: "Convert the childnode's return to False",
};

export default function Control(props) {
  const [payloadValue, setPayloadValue] = useState(
    props.selected.data.ros_payload.data
  );

  const debug = (e) => {
    console.log(props.selected.data.ros_payload);
    console.log(payloadValue);
  };

  return (
    <React.Fragment>
      <p>{`Description: ${controlNodeDescription[payloadValue]}`}</p>
      <Button
        className="mt-2"
        style={{ width: "20%", marginLeft: "10px" }}
        onClick={props.teachDelete}
        variant="danger"
      >
        Remove
      </Button>
      <Button
        className="mt-2"
        style={{ width: "20%", marginLeft: "10px" }}
        onClick={debug}
        variant="danger"
      >
        Debug
      </Button>
    </React.Fragment>
  );
}
