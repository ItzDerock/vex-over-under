#include <fstream>
#include <map>
#include <sstream>

#include "main.h"
#include "robot/odom.hpp"
#include "robot/position.hpp"

// the global cache ([key: file]: path)
std::map<std::string, std::shared_ptr<std::vector<odom::RobotPosition>>> cache;

/**
 * @brief function that returns elements in a file line, separated by a
 * delimeter
 *
 * @param input the raw string
 * @param delimeter string separating the elements in the line
 * @param output vector to store the elements in
 * @return std::vector<std::string> array of elements read from the file
 */
static void readElement(const std::string& input, std::string delimiter,
                        std::vector<std::string>& output) {
  std::string token;
  std::string s = input;
  size_t pos = 0;

  // main loop
  while ((pos = s.find(delimiter)) !=
         std::string::npos) {  // while there are still delimiters in the string
    token = s.substr(0, pos);  // processed substring
    output.push_back(token);
    s.erase(0, pos + delimiter.length());  // remove the read substring
  }

  output.push_back(s);  // add the last element to the returned string
}

/**
 * @brief Get a path from the sd card
 *
 * @param filePath The file to read from
 * @param output The vector to store the path in
 */
static void getData(const std::string& path,
                    std::shared_ptr<std::vector<odom::RobotPosition>> output) {
  std::string line;
  std::vector<std::string> pointInput;
  odom::RobotPosition pathPoint(0, 0, 0);

  // read path
  std::ostringstream buf;
  std::ifstream input(path);
  buf << input.rdbuf();
  std::string data = buf.str();

  // format data from the asset
  std::vector<std::string> dataLines;
  readElement(data, "\n", dataLines);

  // read the points until 'endData' is read
  for (std::string line : dataLines) {
    if (line == "endData" || line == "endData\r") break;
    if (line.rfind("#", 0) == 0) continue;  // ignore comments
    if (line == "") continue;               // ignore empty lines

    // parse
    pointInput.clear();
    readElement(line, ",", pointInput);

    pathPoint.x = std::stof(pointInput.at(0));      // x position
    pathPoint.y = std::stof(pointInput.at(1));      // y position
    pathPoint.theta = std::stof(pointInput.at(2));  // velocity

    // save
    output->push_back(pathPoint);
  }

  // close file
  input.close();
}

static pros::Mutex fileLoadMutex;

void odom::loadPaths(std::vector<std::string> const& files) {
  // lock mutex
  fileLoadMutex.take();

  for (std::string file : files) {
    // load the path
    std::shared_ptr<std::vector<odom::RobotPosition>> path =
        std::make_shared<std::vector<odom::RobotPosition>>();

    getData(file, path);

    std::cout << "loaded path: " << file << std::endl
              << "  size: " << path->size() << std::endl;

    // cache the path
    cache[file] = path;
  }

  // unlock mutex
  fileLoadMutex.give();
}

std::shared_ptr<std::vector<odom::RobotPosition>> odom::getPath(
    std::string const& pathName) {
  fileLoadMutex.take();
  auto path = cache[pathName];
  fileLoadMutex.give();

  return path;
}