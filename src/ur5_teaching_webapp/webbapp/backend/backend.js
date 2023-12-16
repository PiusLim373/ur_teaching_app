//express
const express = require("express");
const app = express();
const port = 5000;

//cors
const cors = require("cors");
const corsOptions = {
  origin: "*",
  credentials: true, //access-control-allow-credentials:true
  optionSuccessStatus: 200,
};

app.use(cors(corsOptions));

//import bodyparser for json handling
const bodyParser = require("body-parser");
app.use(bodyParser.json());

//connect to database
const mongoose = require("mongoose");
mongoose.connect("mongodb://127.0.0.1:27017/ur5_teaching_webapp", () => {
  console.log("connected to database");
});

//middleware
const teachRouter = require("./routes/teach");
app.use("/api/teach", teachRouter);

const missionControlRouter = require("./routes/mission_control");
app.use("/api/mission_control", missionControlRouter);

//routes
app.get("/", (req, res) => {
  console.log("good afernoon");
  res.status(200).send("good afternnon to you");
});

app.listen(port, () => {
  console.log(`listening at ${port}`);
});
