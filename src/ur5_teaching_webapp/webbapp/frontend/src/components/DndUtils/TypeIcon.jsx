import React from "react";
import FolderIcon from "@mui/icons-material/Folder";
import ImageIcon from "@mui/icons-material/Image";
import ListAltIcon from "@mui/icons-material/ListAlt";
import DescriptionIcon from "@mui/icons-material/Description";

import ArrowForwardIcon from "@mui/icons-material/ArrowForward";
import QuestionMarkIcon from "@mui/icons-material/QuestionMark";
import StartIcon from "@mui/icons-material/Start";
import CheckCircleIcon from "@mui/icons-material/CheckCircle";
import CancelIcon from "@mui/icons-material/Cancel";

import KeyboardIcon from "@mui/icons-material/Keyboard";
import PrecisionManufacturingIcon from "@mui/icons-material/PrecisionManufacturing";
import CameraAltIcon from "@mui/icons-material/CameraAlt";

export const TypeIcon = (props) => {
  if (props.droppable) {
    switch (props.fileType) {
      case "sequence":
        return <ArrowForwardIcon />;
      case "fallback":
        return <QuestionMarkIcon />;
      case "start":
        return <StartIcon />;
      case "succeeder":
        return <CheckCircleIcon />;
      case "failer":
        return <CancelIcon />;
      default:
        return null;
    }
  }

  switch (props.fileType) {
    case "saved_joint":
    case "saved_cartesian":
    case "custom_joint":
    case "custom_cartesian":
    case "jog":
      return <PrecisionManufacturingIcon />;
    case "vision_service":
      return <CameraAltIcon />;
    case "plc_service":
      return <KeyboardIcon />;

    default:
      return null;
  }
};
