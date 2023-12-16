import React from "react";
import { useState, useEffect, useRef } from "react";
import Typography from "@mui/material/Typography";
import ArrowRightIcon from "@mui/icons-material/ArrowRight";
import ErrorIcon from "@mui/icons-material/Error";
import { TypeIcon } from "./TypeIcon";
import styles from "./CustomNode.module.css";
import {
  Button,
  OverlayTrigger,
  Tooltip,
  Overlay,
  Popover,
} from "react-bootstrap";

export const CustomNode = (props) => {
  const { id, droppable, data } = props.node;
  const [show, setShow] = useState(false);
  const target = useRef(null);
  const indent = props.depth * 12;

  useEffect(() => {
    // force to expand all by default
    if (!props.isOpen && droppable) props.onToggle();
  });

  const handleToggle = (e) => {
    e.stopPropagation();
    props.onToggle(props.node.id);
  };

  const handleClick = (e) => {
    let data = { id: id, childProps: props };
    props.onSelect(data);
  };

  return (
    <div
      className={`tree-node ${styles.root} ${
        props.displayStatusColor && data.status === "queued"
          ? styles.rootSelected
          : props.displayStatusColor && data.status === "completed"
          ? styles.rootSucceeded
          : props.displayStatusColor && data.status === "errorred"
          ? styles.rootErrorred
          : !props.displayStatusColor && props.selected
          ? styles.rootSelected
          : ""
      }`}
      style={{ paddingInlineStart: indent }}
      onClick={handleClick}
    >
      <div
        className={`${styles.expandIconWrapper} ${
          props.isOpen ? styles.isOpen : ""
        }`}
      >
        {props.node.droppable && (
          <div onClick={handleToggle}>
            <ArrowRightIcon />
          </div>
        )}
      </div>
      <div>
        <TypeIcon droppable={droppable || false} fileType={data?.fileType} />
      </div>
      <div className={styles.labelGridItem}>
        <Typography variant="body2">
          {`${props.node.text} `}
          {!props.node.data.valid || !props.node.data.payloadDataValid ? (
            <React.Fragment>
              <ErrorIcon
                style={{ color: "#d9534f" }}
                ref={target}
                onMouseEnter={() => setShow(true)}
                onMouseLeave={() => setShow(false)}
              />
              <Overlay target={target.current} show={show} placement="right">
                {({ placement, arrowProps, show: _show, popper, ...props }) => (
                  <div
                    {...props}
                    style={{
                      position: "absolute",
                      backgroundColor: "rgba(255, 100, 100, 0.85)",
                      padding: "2px 10px",
                      color: "white",
                      borderRadius: 3,
                      ...props.style,
                    }}
                  >
                    {data.invalidMsg !== ""
                      ? data.invalidMsg
                      : !data.payloadDataValid
                      ? "Payload data invalid"
                      : ""}
                  </div>
                )}
              </Overlay>
            </React.Fragment>
          ) : (
            <span
              style={{ fontStyle: "italic", color: "grey" }}
            >{`- ${props.node.data.ros_payload.data}`}</span>
          )}
        </Typography>
      </div>
    </div>
  );
};
