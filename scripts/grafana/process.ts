import { readFile } from "fs/promises";

export async function processIterator(
  contents: Iterator<string>,
  onData?: (data: Record<string, unknown>) => void
) {
  let data = {} as Record<string, unknown>;
  const output = [] as Record<string, unknown>[];

  let time = 0;
  let next = contents.next();

  while (!next.done) {
    const [key, value] = (next.value as string).split(": ");
    next = contents.next();
    if (!key || !value) continue;

    let dataKey: string = key;
    let dataValue: unknown;

    switch (key) {
      case "pathPoints.size()": {
        output.push(data);
        data = {};
        break;
      }

      case "pose":
        data["time"] = time += 10; // milliseconds
      case "lookaheadPose":
        const [x, y, z] = value.split(", ").map((v) => parseFloat(v));
        dataValue = { x, y, z };

        break;

      case "actualLeftVel":
      case "actualRightVel":
        const [real, target] = value.split(" / ").map((v) => parseFloat(v));
        // data[key] = { real, target };
        dataValue = { real, target };

        break;

      // ignore these
      case "x":
      case "y":
      case "theta":
        continue;

      default:
        let float = parseFloat(value);
        dataValue = isNaN(float) ? value : float;
    }

    if (dataKey in data) {
      onData?.(data);
      output.push(data);
      // await new Promise((resolve) => setTimeout(resolve, 10));
      data = {
        [dataKey]: dataValue,
      };
    } else {
      data[dataKey] = dataValue;
    }
  }

  return output;
}

export async function readFileToIterator(filename: string) {
  console.log(`Processing ${filename}`);
  // const file = await Bun.file(filename).text();
  const file = await readFile(filename, "utf-8");

  // Parse the file
  const lines = file.split("\n");

  // skip to first 'pathPoints.size' line
  while (lines.length > 0 && !lines[0].startsWith("pathPoints.size")) {
    lines.shift();
  }

  console.log(`Found ${lines.length} lines`);
  return lines.values();
}

// if main
if (import.meta.main) {
  const filename = process.argv[2];
  const output = process.argv[3];
  const data = await processIterator(await readFileToIterator(filename));

  if (output) {
    await Bun.write(Bun.file(output), JSON.stringify(data));
  } else {
    console.log(JSON.stringify(data));
  }
}
