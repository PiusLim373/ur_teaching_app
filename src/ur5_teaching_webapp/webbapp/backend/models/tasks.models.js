const mongoose = require("mongoose");
const Schema = mongoose.Schema;

const tasksScheme = new Schema(
  {
    task: {
      type: Array,
    },
  },
  { strict: false }
);

// const noSchema = new Schema({}, {strict: false});

const tasks = mongoose.model("tasks", tasksScheme);
module.exports = tasks;
