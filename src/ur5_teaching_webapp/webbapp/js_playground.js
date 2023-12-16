let payload = [
  {
    id: 1,
    parent: null,
    children: [
      {
        id: 3,
        parent: 1,
        children: [],
        pose: [300, 400],
        type: "sequence",
      },
      {
        id: 4,
        parent: 1,
        children: [
          {
            id: 5,
            parent: 1,
            children: [],
            pose: [900, 990],
            type: "sequence",
          },
        ],
        pose: [500, 600],
        type: "sequence",
      },
    ],
    pose: [100, 100],
    type: "start",
  },
  {
    id: 2,
    parent: null,
    children: [],
    pose: [700, 800],
    type: "fallback",
  },
];

// const func = (payload) => {
//   payload.map((node) => {
//     console.log(node.pose);
//     func(node.children);
//   });
// };

const func = (data) => {
  console.log(data);
};

payload.map((node) => {
  func(node);
});
