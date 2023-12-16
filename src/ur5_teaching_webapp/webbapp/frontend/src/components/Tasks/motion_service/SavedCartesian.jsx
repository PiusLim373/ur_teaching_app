import React, { useState, useEffect } from "react";
import { Button, InputGroup, Form } from "react-bootstrap";
import axios from "axios";
import globalVars from "../../../globalVars";

export default function SavedCartesian(props) {
  const available_cartesian = ["dispose_box"];
  const [payloadVerified, setPayloadVerified] = useState(
    props.selected.data.payloadDataValid
  );
  const [payloadValue, setPayloadValue] = useState(
    props.selected.data.ros_payload.data
  );

  useEffect(() => {
    let newData = props.selected;
    newData.data.ros_payload.data = payloadValue;
    newData.data.payloadDataValid = payloadVerified;
    props.teachChange(newData);
  }, [payloadValue]);

  const debug = (e) => {
    console.log(props.selected.data.ros_payload);
    console.log(payloadValue);
  };

  const verifyPayload = (event) => {
    console.log(payloadValue);
    console.log(event.target.value);
    try {
      if (event.target.value !== "placeholder") {
        setPayloadVerified(true);
      } else setPayloadVerified(false);
      setPayloadValue(event.target.value);
      console.log("sucessfully set payloadvalue");
    } catch (err) {
      console.log("catch" + err);
      setPayloadVerified(false);
      setPayloadValue("");
    }
  };
  const testMotion = () => {
    if (payloadVerified) {
      let data = {
        _id: "ui_issued_testing",
        task: [
          {
            id: -1,
            parent: -1,
            text: "",
            droppable: false,
            data: {
              fileType: "",
              valid: false,
              status: "",
              selected: false,
              ros_payload: {
                service: "motion_service",
                task: "saved_cartesian",
                data: payloadValue,
              },
            },
          },
        ],
      };
      alert("Motion will be executed");
      axios
        .post(
          "http://" + globalVars.ip + ":5000/api/mission_control/motion_test/",
          data
        )
        .then((res) => {
          if (res.data) {
            if (res.data.success) alert("Motion completed successfully");
            else alert("Invalid payload! Please check");
          }
        });
    } else alert("Invalid payload! Please check");
  };

  return (
    <React.Fragment>
      {"Description: Go to a pre-saved joint goal"}
      <InputGroup>
        <InputGroup.Text id="basic-addon1">Payload</InputGroup.Text>
        <Form.Select
          aria-label="Default select example"
          onChange={verifyPayload}
          isInvalid={!payloadVerified}
          value={props.selected.data.ros_payload.data}
        >
          <option value="placeholder">Select a saved joint</option>
          {available_cartesian.map((joint) => (
            <option key={joint} value={joint}>
              {joint}
            </option>
          ))}
        </Form.Select>
      </InputGroup>

      <Button
        className="mt-2"
        style={{ width: "20%" }}
        onClick={testMotion}
        variant="outline-danger"
      >
        Test
      </Button>
      <Button
        className="mt-2"
        style={{ width: "20%", marginLeft: "10px" }}
        onClick={props.teachDelete}
        variant="danger"
      >
        Remove
      </Button>
    </React.Fragment>
  );
}
