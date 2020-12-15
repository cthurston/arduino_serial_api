import 'dart:async';
import 'dart:convert';
import 'dart:io';
import 'dart:typed_data';

import 'package:dart_serial_port/dart_serial_port.dart';

const libraryVersion = 'V0.8';
const encoder = Utf8Encoder();
const decoder = Utf8Decoder();
const int DIGITAL_HIGH = 1;
const int DIGITAL_LOW = 0;
const int PIN_INPUT = 0;
const int PIN_OUTPUT = 1;
const serialTimeoutMs = 1500;

/// Build a command string that can be sent to the arduino.
/// Input:
///   cmd (str): the command to send to the arduino
///   args (iterable): the arguments to send to the command
Uint8List buildCmdString(String cmd, [List<dynamic> args = const []]) {
  var a = args.join('%');
  return encoder.convert('@$cmd%$a\$!');
}

String readAvailableLine(SerialPort sp) {
  // while (sp.bytesAvailable == 0) {
  //   sleep(Duration(milliseconds: 1));
  // }
  // var bytes = sp.bytesAvailable;
  print(sp.bytesAvailable);
  var buf = sp.read(sp.bytesAvailable, timeout: serialTimeoutMs);
  var line = decoder.convert(buf);
  return line.replaceAll('\r\n', '');
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
  int timeout = 2000;
  int baudRate;
  late SerialPort sp;
  late SPI spi;
  // late SerialPortReader reader;
  String message = '';
  Completer _serialResponse = Completer();

  Arduino({
    this.baudRate = 115200,
    String? port,
    SerialPort? serialPort,
  }) {
    if (port == null && serialPort == null) {
      port = findPort();
    }

    sp = serialPort ?? SerialPort(port);

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

    spi = SPI(this);
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

    sp.read(11, timeout: serialTimeoutMs); // should be 'connected'
    sp.flush();

    var version = getSketchVersion();
    print('ver: $version');
    if (libraryVersion != version) {
      throw Exception(
          'The Arduino sketch is not the correct version, ${libraryVersion}.');
    }
  }

  String getSketchVersion() {
    var cmd = buildCmdString('version');
    sp.flush();
    sp.write(cmd, timeout: serialTimeoutMs);
    sp.drain();

    var version = readline();
    return version;
  }

  String readline() {
    var timeout = Duration(milliseconds: 300);
    var completed = false;
    var line = '';
    while (!completed && DateTime.now().isBefore(DateTime.now().add(timeout))) {
      var buf = sp.read(sp.bytesAvailable, timeout: serialTimeoutMs);
      line = decoder.convert(buf);
      if (line.contains('\r\n')) completed = true;
    }

    return line.replaceAll('\r\n', '');
  }

  /// Sets I/O mode of pin
  /// [mode] should be PIN_INPUT or PIN_OUTPUT
  void pinMode(int pin, int mode) {
    if (mode == PIN_INPUT) {
      pin = -pin;
    }
    var cmd = buildCmdString('pm', [pin, mode]);
    sp.flush();
    sp.write(cmd);
  }

  /// Send digitalWrite command to a pin
  void digitalWrite(int pin, int value) {
    if (value == DIGITAL_LOW) {
      pin = -pin;
    }
    var cmd = buildCmdString('dw', [pin]);
    sp.flush();
    sp.write(cmd);
  }

  /// return 1 for HIGH, 0 for LOW
  int digitalRead(int pin) {
    var cmd = buildCmdString('dr', [pin]);
    sp.flush();
    sp.write(cmd, timeout: serialTimeoutMs);
    return int.parse(readline());
  }

  void analogWrite(int pin, int value) {
    var cmd = buildCmdString('aw', [pin, value]);
    sp.flush();
    sp.write(cmd, timeout: serialTimeoutMs);
  }

  int analogRead(int pin) {
    var cmd = buildCmdString('ar', [pin]);
    sp.flush();
    sp.write(cmd, timeout: serialTimeoutMs);
    return _readLastInt();
  }

  int _readLastInt() {
    // the analog value can only be up to 4095?
    var bytes = sp.bytesAvailable;
    Uint8List buf = sp.read(bytes, timeout: serialTimeoutMs);
    var line = decoder.convert(buf);
    var ans = int.parse(line.replaceAll('\r\n', ''));
    return ans;
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

  SerialPort get sp {
    return board.sp;
  }

  SPI(this.board);

  void initialize(int syncPin) {
    var cmd = buildCmdString('spii', [syncPin]);
    sp.write(cmd);
    sp.flush();
    spiInitialized = true;
  }

  void write(int b1, int b2) {
    if (!spiInitialized) {
      throw Exception('SPI must be intialized first.');
    }

    var cmd = buildCmdString('spiw', [b1, b2]);
    sp.write(cmd);
    sp.flush();
  }
}
