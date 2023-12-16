import React, { useState, useEffect, useRef } from "react";
import { Button, InputGroup, Form } from "react-bootstrap";
import axios from "axios";
import globalVars from "../../../globalVars";

export default function CustomCartesian(props) {
  const [payloadVerified, setPayloadVerified] = useState(
    props.selected.data.payloadDataValid
  );
  const [rawPayload, setRawPayload] = useState(
    props.selected.data.ros_payload.data.toString()
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
    console.log(props.selected.data.ros_payload.data.length);
  };

  const verifyPayload = (event) => {
    console.log(event.target.value);
    setRawPayload(event.target.value);
    try {
      let cartesian_list = JSON.parse("[" + event.target.value + "]");
      if (cartesian_list.length === 7) {
        setPayloadVerified(true);
        setPayloadValue(cartesian_list);
      } else {
        setPayloadVerified(false);
        setPayloadValue([]);
      }
    } catch (err) {
      setPayloadVerified(false);
      setPayloadValue([]);
    }
  };

  const getCurrentPose = () => {
    axios
      .get("http://" + globalVars.ip + ":5000/api/teach/get_pose/")
      .then((res) => {
        if (res.data) {
          let event = {
            target: {
              value: JSON.stringify(res.data.current_pose).replace(
                /[\[\]']+/g,
                ""
              ),
            },
          };
          verifyPayload(event);
        } else {
          alert("Error getting pose");
        }
      });
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
              payloadDataValid: false,
              status: "",
              selected: false,
              ros_payload: {
                service: "motion_service",
                task: "custom_cartesian",
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
      {"Description: Go to the joint goal as specified"}
      <InputGroup>
        <InputGroup.Text id="basic-addon1">Payload</InputGroup.Text>
        <Form.Control
          placeholder="List of 7 Cartesian coordinates, seperated by comma"
          onChange={verifyPayload}
          isInvalid={!payloadVerified}
          value={rawPayload}
          // value={props.selected.data.ros_payload.data}
        />
      </InputGroup>
      <Button
        className="mt-2"
        style={{ width: "30%" }}
        onClick={getCurrentPose}
        variant="warning"
      >
        Get Current
      </Button>
      <Button
        className="mt-2"
        style={{ width: "20%", marginLeft: "10px" }}
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
      <Button
        className="mt-2"
        style={{ width: "20%", marginLeft: "10px" }}
        onClick={debug}
        variant="danger"
      >
        Debug
      </Button>
      <p>{props.teachDebug}</p>
    </React.Fragment>
  );
}
