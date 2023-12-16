import "./App.css";
import "bootstrap/dist/css/bootstrap.css";
import Header from "./components/Header";
import { BrowserRouter, Routes, Route } from "react-router-dom";
import Teach from "./components/Teach";
import MissionControl from "./components/MissionControl";
import Deploy from "./components/Deploy";
import { useState } from "react";
import { textFieldClasses } from "@mui/material";

function App() {

  // TODO: fix the toast logic!!
  const [showToast, setShowToast] = useState(false);
  const [toastText, setToastText] = useState("");

  const handleToastShow = (show) => {
    console.log("app js", show);
    if(show){
      setShowToast(show);
      setTimeout(() => {
        setShowToast(false);
      }, 3000);
    }
  };

  const handleToastText = (text) => {
    console.log("app js", text);
    setToastText(text);
  };
  return (
    <div className="container">
      <BrowserRouter>
        <Header handleToastShow={handleToastShow} showToast={showToast} toastText={toastText} />
        <br />
        <Routes>
          <Route
            path="/"
            element={
              <Teach
                handleToastShow={handleToastShow}
                handleToastText={handleToastText}
              />
            }
          />
          <Route path="/deploy" element={<Deploy />} />
          <Route path="/mission_control" element={<MissionControl />} />
          <Route path="/dnd_test" element={<Teach />} />
        </Routes>
      </BrowserRouter>
    </div>
  );
}

export default App;
