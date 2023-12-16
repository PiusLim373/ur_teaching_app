const router = require("express").Router();
const tasks_model = require("../models/tasks.models");

// const rosnodejs = require("rosnodejs");
// rosnodejs.initNode("/rosnodejs");
// const nh = rosnodejs.nh;

// const client = nh.serviceClient(
//   "/tree_executor",
//   "tree_executor/TreeExecutorService"
// );
// const tree_executor_service =
//   rosnodejs.require("tree_executor").srv.TreeExecutorService;

var active_mission_id = "";

router.get("/", async (req, res) => {
  res.status(200).json({ activeMissionID: active_mission_id });
});

router.get("/activate/:id", async (req, res) => {
  if (active_mission_id === "") {
    active_mission_id = req.params.id;
    let filter = {
      _id: req.params.id,
      // "task.$[].data": { $elemMatch: { status: { $ne: "" } } },
      task: { $elemMatch: { "data.status": { $ne: "" } } },
    };
    let update_key_str = "task.$[].data.status";
    let target = { [update_key_str]: "" };
    let postHandler = await tasks_model.updateMany(filter, {
      $set: target,
    });
    console.log(postHandler);
    postHandler = await tasks_model.findOne({ _id: req.params.id });
    console.log(postHandler);
    const request = new tree_executor_service.Request();
    request.json_data = JSON.stringify(postHandler);
    client.call(request).then((resp) => {
      res.status(200).json({ hasActivated: resp.success, data: postHandler });
    });
  } else {
    res.status(200).json({ hasActivated: false });
  }
});

router.get("/detail/:id", async (req, res) => {
  const postHandler = await tasks_model.findOne({
    _id: req.params.id,
  });
  res.status(200).json(postHandler);
});

router.post("/detail/:id", async (req, res) => {
  let filter = { _id: req.params.id, "task.id": req.body.task_id };
  let update_key_str = "task.$.data.status";
  let target = { [update_key_str]: req.body.task_status };
  const postHandler = await tasks_model.findOneAndUpdate(filter, {
    $set: target,
  });
  res.status(200).json(postHandler);
});

router.get("/terminate", async (req, res) => {
  //send a stop signal to ros
  active_mission_id = "";
  res.status(200).json({ message: "ok" });
});

router.post("/motion_test", async (req, res) => {
  const request = new tree_executor_service.Request();
  request.json_data = JSON.stringify(req.body);
  client.call(request).then((resp) => {
    res.status(200).json({ success: resp.success });
  });
});

router.get("/debug", async (req, res) => {
  // client.call({ json_data: "1244" }).then((resp) => {
  //   console.log(resp);
  //   res.status(200).json({ message: resp });
  // });
  const request = new tree_executor_service.Request();
  request.json_data = JSON.stringify({
    _id: "1234565",
    task: [
      {
        id: 0,
        name: "Saved Joint 0",
        service: "motion_server",
        task: "saved_joint",
        payload: "home",
        status: "",
      },
      {
        id: 1,
        name: "Saved Joint 1",
        service: "motion_server",
        task: "saved_joint",
        payload: "sparebay_home",
        status: "created",
      },
      {
        id: 2,
        name: "Custom Joint 2",
        service: "motion_server",
        task: "custom_joint",
        payload: [0, -1.2, -1.7, 4.2, 1.6, 0],
        status: "created",
      },
      {
        id: 3,
        name: "Saved Cartesian 3",
        service: "motion_server",
        task: "saved_cartesian",
        payload: "dispose_box",
        status: "created",
      },
      {
        id: 4,
        name: "Custom Cartesian 4",
        service: "motion_server",
        task: "custom_cartesian",
        payload: [-0.3, 0.2, 0.7, 0.707, 0.707, 0, 0],
        status: "created",
      },
    ],
  });
  client.call(request).then((resp) => {
    console.log(resp.success);
    res.status(200).json({ message: resp });
  });
});

module.exports = router;
