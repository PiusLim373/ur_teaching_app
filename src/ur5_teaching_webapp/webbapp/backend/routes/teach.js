const router = require("express").Router();
const tasks_model = require("../models/tasks.models");

// const rosnodejs = require("rosnodejs");
// rosnodejs.initNode("/rosnodejs");
// const nh = rosnodejs.nh;

// const client = nh.serviceClient("/motion_server", "ur5_pap/MotionService");
// const motion_service = rosnodejs.require("ur5_pap").srv.MotionService;

var edit_saved_id = "";

router.get("/", async (req, res) => {
  try {
    console.log(edit_saved_id);
    let return_data = {};
    if (edit_saved_id !== "") {
      return_data = await tasks_model.findOne({ _id: edit_saved_id });
      console.log(return_data);
    } else return_data = {};
    res.status(200).json({
      task_to_edit: return_data,
    });
  } catch (err) {
    res.status(400).json({ message: err });
  }
});

router.post("/", async (req, res) => {
  try {
    console.log(req.body.edit_saved_id);
    edit_saved_id = req.body.edit_saved_id;
    res.status(200).json({
      success: true,
    });
  } catch (err) {
    res.status(400).json({ message: err });
  }
});

router.get("/get_saved", async (req, res) => {
  try {
    const postHandler = await tasks_model.find();
    res.status(200).json({
      saved_tasks: postHandler,
    });
  } catch (err) {
    res.status(400).json({ message: err });
  }
});

router.post("/save", async (req, res) => {
  console.log(req.body.taskID);
  if (req.body.taskID !== "") {
    let filter = { _id: req.body.taskID };
    let update = { task: req.body.task, name: req.body.name };
    const postHandler = await tasks_model.findOneAndUpdate(filter, update);
    console.log(postHandler);
    res.status(200).json({ message: "ok" });
  } else {
    const postHandler = await tasks_model.insertMany(req.body);
    // console.log(postHandler);
    res.status(200).json({ message: "ok" });
  }
});

router.get("/get_pose", async (req, res) => {
  const request = new motion_service.Request();
  request.motion_task = "status";
  client.call(request).then((resp) => {
    let processed_joint = resp.current_joint.map((joint) =>
      parseFloat(joint.toFixed(3))
    );
    let processed_pose = resp.current_pose.map((pose) =>
      parseFloat(pose.toFixed(3))
    );
    res.status(200).json({
      current_joint: processed_joint,
      current_pose: processed_pose,
    });
  });
});

module.exports = router;
