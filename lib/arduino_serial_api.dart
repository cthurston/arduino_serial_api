import 'dart:io';
import 'dart:async';
import 'dart:convert';
import 'dart:typed_data';

import 'package:dart_serial_port/dart_serial_port.dart';

const libraryVersion = 'V0.8';
const encoder = Utf8Encoder();
const decoder = Utf8Decoder();
const int DIGITAL_HIGH = 1;
const int DIGITAL_LOW = 0;
const int PIN_INPUT = 0;
const int PIN_OUTPUT = 1;
const serialTimeoutMs = 500;
const Duration readTimeout = Duration(milliseconds: 500);

/// Build a command string that can be sent to the arduino.
/// Input:
///   cmd (str): the command to send to the arduino
///   args (iterable): the arguments to send to the command
Uint8List buildCmdString(String cmd, [List<dynamic> args = const []]) {
  var a = args.join('%');
  return encoder.convert('@$cmd%$a\$!');
}

void listAvailablePorts() {
  var i = 0;
  for (var name in SerialPort.availablePorts) {
    try {
      var sp = SerialPort(name);
      print('${++i}) $name');
      print('\tDescription: ${sp.description}');
      print('\tManufacturer: ${sp.manufacturer}');
      print('\tSerial Number: ${sp.serialNumber}');
      sp.dispose();
    } catch (e) {}
  }
}

String findPort() {
  var port = '';
  var arduinoMatcher = RegExp(r'arduino', caseSensitive: false);
  for (var name in SerialPort.availablePorts) {
    try {
      var sp = SerialPort(name);
      if (sp.manufacturer != null && sp.manufacturer.contains(arduinoMatcher)) {
        port = name;
        sp.dispose();
        break;
      }
    } catch (e) {}
  }

  return port;
}

class Arduino {
  int baudRate;
  late SerialPort sp;
  late SPI spi;
  late Stepper stepper;
  String message = '';

  Arduino({
    this.baudRate = 115200,
    String? port,
    SerialPort? serialPort,
  }) {
    if (port == null && serialPort == null) {
      port = findPort();
    }

    sp = serialPort ?? SerialPort(port);

    spi = SPI(this);
    stepper = Stepper(this);
  }

  Future<dynamic> start() async {
    var open = sp.openReadWrite();
    if (!open) {
      throw Exception(
          'Could not connect to the Arduino. ${SerialPort.lastError}');
    }

    var config = SerialPortConfig();
    config.baudRate = baudRate;
    config.bits = 8;
    config.stopBits = 1;
    config.parity = SerialPortParity.none;
    config.flowControl = SerialPortFlowControl.none;

    sp.config = config;

    config.dispose();

    sp.read(11, timeout: 2000); // should be 'connected'
    sp.flush();

    var version = getSketchVersion();
    print('Arduino Sketch Version: $version');
    if (libraryVersion != version) {
      throw Exception(
          'The Arduino sketch is not the correct version, ${libraryVersion}.');
    }
  }

  String readline({Duration timeout = readTimeout}) {
    var completed = false;
    var line = '';
    var expires = DateTime.now().add(timeout);
    while (!completed && DateTime.now().isBefore(expires)) {
      if (sp.bytesAvailable > 0) {
        var buf = sp.read(sp.bytesAvailable, timeout: serialTimeoutMs);
        line += decoder.convert(buf);
        if (line.contains('\r\n')) completed = true;
      }
    }

    return line.replaceAll('\r\n', '');
  }

  void write(String cmd, [List<dynamic> args = const []]) {
    var buf = buildCmdString(cmd, args);
    sp.write(buf, timeout: serialTimeoutMs);
    sp.flush();
  }

  String getSketchVersion() {
    write('version');

    var version = readline();
    return version;
  }

  /// Sets I/O mode of pin
  /// [mode] should be PIN_INPUT or PIN_OUTPUT
  void pinMode(int pin, int mode) {
    if (mode == PIN_INPUT) {
      pin = -pin;
    }
    write('pm', [pin, mode]);
  }

  /// Send digitalWrite command to a pin
  void digitalWrite(int pin, int value) {
    if (value == DIGITAL_LOW) {
      pin = -pin;
    }
    write('dw', [pin]);
  }

  /// return 1 for HIGH, 0 for LOW
  int digitalRead(int pin) {
    write('dr', [pin]);
    return int.parse(readline());
  }

  void analogWrite(dynamic pin, int value) {
    write('aw', [pin, value]);
  }

  int analogRead(int pin) {
    write('ar', [pin]);
    var r = readline();
    return int.parse(r);
  }

  void close() {
    if (sp.isOpen) {
      sp.flush();
      sp.close();
    }
  }
}

/// Write to the SPI
class SPI {
  Arduino board;
  bool spiInitialized = false;

  SPI(this.board);

  void initialize(int syncPin) {
    board.write('spii', [syncPin]);
    spiInitialized = true;
  }

  void transfer(int b1, int b2) {
    if (!spiInitialized) {
      throw Exception('SPI must be intialized first.');
    }
    board.write('spiw', [b1, b2]);
  }
}

/// Stepper motor controlling.
class Stepper {
  Arduino board;
  bool stepperInitialized = false;
  SerialPort get sp {
    return board.sp;
  }

  Stepper(this.board);

  void initialize(int speed, int position) {
    board.write('stpi', [speed, position]);
    stepperInitialized = true;
  }

  void step(int steps) {
    board.write('stps', [steps]);
  }

  void stepTo(int position) {
    board.write('stpgo', [position]);
  }

  void setSpeed(int speed) {
    board.write('stpsp', [speed]);
  }

  int getSpeed() {
    board.write('rstpsp', []);
    return int.parse(board.readline());
  }

  int getStep() {
    board.write('rstps', []);
    return int.parse(board.readline());
  }
}

// reader = SerialPortReader(sp, timeout: serialTimeoutMs);
// reader.stream.map((d) => decoder.convert(d)).listen((data) {
//   print('some data $data');

//   message += data;

//   if (message.contains('\r\n')) {
//     var parts = message.split('\r\n');
//     _serialResponse.complete(parts.first);

//     message = parts.length > 1 ? parts[1] : '';
//   }
// });
