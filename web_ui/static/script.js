const canvas = document.getElementById("map");
const ctx = canvas.getContext("2d");
const chatLog = document.getElementById("chatLog");
const loader = document.getElementById("loader");
const promptDiv = document.getElementById("prompt");
ctx.font = "12px sans-serif";
const promptInput = document.getElementById("promptInput");

var robots = {};
var stations = {};
var coord_multiplier = 1.5;

var ros = new ROSLIB.Ros({
  url: "ws://192.168.1.133:9090",
});

ros.on("connection", function () {
  console.log("Connected to websocket server.");
});

ros.on("error", function (error) {
  console.log("Error connecting to websocket server: ", error);
});

ros.on("close", function () {
  console.log("Connection to websocket server closed.");
});

var prompt = new ROSLIB.Topic({
  ros: ros,
  name: "/prompt",
  messageType: "std_msgs/String",
});

var response = new ROSLIB.Topic({
  ros: ros,
  name: "/response",
  messageType: "std_msgs/String",
});

response.subscribe(function (message) {
  appendToChat("🤖 " + message.data);
  loader.classList.add("hidden");
  promptDiv.classList.remove("hidden");
});

var robot_status = new ROSLIB.Topic({
  ros: ros,
  name: "/robot_status",
  messageType: "std_msgs/String",
});

robot_status.subscribe(function (message) {
  let robot_status_data = JSON.parse(message.data);
  robots[robot_status_data.robot_id] = {
    x: Math.round(robot_status_data.y * 100 * coord_multiplier + 250),
    y: Math.round(robot_status_data.x * 100 * coord_multiplier + 250),
    available: robot_status_data.available,
  };
  drawMap();
});

var station_status = new ROSLIB.Topic({
  ros: ros,
  name: "/station_status",
  messageType: "std_msgs/String",
});

station_status.subscribe(function (message) {
  let station_status_data = JSON.parse(message.data);
  stations[station_status_data.station_id] = {
    x: Math.round(station_status_data.y * 100 * coord_multiplier + 250),
    y: Math.round(station_status_data.x * 100 * coord_multiplier + 250),
    item: station_status_data.item || "-",
  };
  drawMap();
});

promptInput.addEventListener("keydown", function (event) {
  if (event.key === "Enter") {
    event.preventDefault();
    sendPrompt();
  }
});

function drawMap() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);

  for (const [id, s] of Object.entries(stations)) {
    ctx.fillStyle = "blue";
    ctx.beginPath();
    ctx.rect(s.x - 10, s.y - 10, 20, 20);
    ctx.fill();
    ctx.fillText(id, s.x + 15, s.y - 3);
    ctx.fillText(
      s.item != "-" ? "Item: " + s.item : "No item",
      s.x + 15,
      s.y + 12
    );
  }

  for (const [id, r] of Object.entries(robots)) {
    ctx.fillStyle = r.available ? "green" : "red";
    ctx.beginPath();
    ctx.arc(r.x, r.y, 10, 0, 2 * Math.PI);
    ctx.fillText(id, r.x + 15, r.y - 3);
    ctx.fill();
    if (!r.available) {
      ctx.fillStyle = "red";
      ctx.fillText("Busy", r.x + 15, r.y + 12);
    } else {
      ctx.fillStyle = "black";
      ctx.fillText("Available", r.x + 15, r.y + 12);
    }
  }
}

function sendPrompt() {
  const input = document.getElementById("promptInput").value;
  const mode = document.getElementById("modeSelect").value;
  const model = document.getElementById("modelSelect").value;
  if (input.trim() !== "") {
    var msg = new ROSLIB.Message({
      data: JSON.stringify({ input, mode, model }),
    });
    prompt.publish(msg);
    appendToChat("👤 " + input);
    document.getElementById("promptInput").value = "";
    loader.classList.remove("hidden");
    promptDiv.classList.add("hidden");
  }
}

function appendToChat(message) {
  const msgDiv = document.createElement("div");
  msgDiv.textContent = message;
  chatLog.appendChild(msgDiv);
  chatLog.scrollTop = chatLog.scrollHeight;
}
