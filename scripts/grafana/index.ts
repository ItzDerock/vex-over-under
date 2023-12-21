import { connectAsync } from "mqtt";
import { join } from "path/posix";
import { processIterator, readFileToIterator } from "./process";

// parse arguments
const args = process.argv.slice(2);
const filename = process.env.FILENAME ?? args[0];
const broker = process.env.BROKER ?? args[1];

if (!filename) {
  console.error("No filename provided");
  process.exit(1);
}

if (!broker) {
  console.error("No broker provided");
  process.exit(1);
}

const client = await connectAsync(broker);
console.log("Connected to broker");

const iterator = await readFileToIterator(filename);
console.log("Processing file");

await processIterator(iterator, (data) => {
  for (const [key, value] of Object.entries(data)) {
    const data = traverse(value);
    for (let [path, value] of data) {
      const mqttKey = join("10ms", "robot", key, ...path);
      if (typeof value != "string") {
        value = JSON.stringify(value);
      }

      client.publish(mqttKey, value as string, {
        retain: true,
        qos: 1,
      });
      console.log("Published", mqttKey, value);
    }
  }
});

console.log("Finished processing");
client.end();

function traverse(data: unknown, path: string[] = []): [string[], unknown][] {
  if (typeof data === "object" && data !== null) {
    let output: [string[], unknown][] = [];
    for (const [key, value] of Object.entries(data)) {
      output.push(...traverse(value, [...path, key]));
    }

    return output;
  }

  return [[path, data]];
}
