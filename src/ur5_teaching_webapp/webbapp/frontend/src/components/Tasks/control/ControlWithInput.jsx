import React, { useState, useEffect } from "react";
import { Button, InputGroup, Form } from "react-bootstrap";

const controlNodeDescription = {
  retry: "Will execute the childnode again if the return is not True",
  wait: "Put a hold on the execution",
};

export default function ControlWithInput(props) {
  const [payloadValue, setPayloadValue] = useState(
    props.selected.data.ros_payload.data
  );
  const [payloadVerified, setPayloadVerified] = useState(
    props.selected.data.payloadDataValid
  );
  const [controlNodeData, setControlNodeData] = useState(
    props.selected.data.ros_payload.control_node_data
  );
  const [rawControlNodeData, setRawControlNodeData] = useState(
    props.selected.data.ros_payload.control_node_data.toString()
  );

  useEffect(() => {
    let newData = props.selected;
    newData.data.ros_payload.control_node_data = controlNodeData;
    newData.data.payloadDataValid = payloadVerified;
    props.teachChange(newData);
  }, [controlNodeData]);

  const debug = (e) => {
    console.log(props.selected.data.ros_payload);
    console.log(payloadValue);
  };

  const verifyPayload = (event) => {
    console.log(event.target.value);
    setRawControlNodeData(event.target.value);
    try {
      let tempData = parseInt(event.target.value);
      if (payloadValue === "retry" && tempData >= 0) {
        //retry shouldnt allow decimal or nagitve decimal or char
        setPayloadVerified(true);
        setControlNodeData(tempData);
      } else if (payloadValue === "wait" && tempData > 0) {
        setPayloadVerified(true);
        setControlNodeData(tempData);
      } else {
        setPayloadVerified(false);
      }
    } catch (err) {
      setPayloadVerified(false);
    }
  };

  return (
    <React.Fragment>
      <p>{`Description: ${controlNodeDescription[payloadValue]}`}</p>
      <InputGroup>
        <InputGroup.Text id="basic-addon1">Payload</InputGroup.Text>
        <Form.Control
          placeholder={
            payloadValue === "retry"
              ? "Numbers of retries if fails, 0 means infinity retry"
              : payloadValue === "wait"
              ? "Duration to wait in seceonds, cannot be 0"
              : ""
          }
          onChange={verifyPayload}
          isInvalid={!payloadVerified}
          value={rawControlNodeData}
        />
      </InputGroup>
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
