import 'dart:convert';
import 'dart:typed_data';

import 'package:dart_serial_port/dart_serial_port.dart';

const libraryVersion = 'V0.7';
const encoder = Utf8Encoder();
const decoder = Utf8Decoder();
const int DIGITAL_HIGH = 1;
const int DIGITAL_LOW = 0;
const int PIN_INPUT = 0;
const int PIN_OUTPUT = 1;

/// Build a command string that can be sent to the arduino.
/// Input:
///   cmd (str): the command to send to the arduino
///   args (iterable): the arguments to send to the command
Uint8List buildCmdString(String cmd, [List<dynamic> args = const []]) {
  var a = args.join('%');
  return encoder.convert('@$cmd%$a\$!');
}

String getSketchVersion(SerialPort sp) {
  var cmd = buildCmdString('version');
  sp.write(cmd);
  sp.flush();
  return readAvailableLine(sp);
}

String readAvailableLine(SerialPort sp) {
  var bytes = 8;
  var buf = sp.read(bytes);
  var line = decoder.convert(buf);
  return line.replaceAll('\r\n', '');
}

// void findPort(int timeout) {
//   var i = 0;
//   for (final name in SerialPort.availablePorts) {
//     final sp = SerialPort(name);
//     sp.openReadWrite();
//     var v = sp.read(5, timeout: timeout);

//     print('${++i}) $name');
//     print('\tDescription: ${sp.description}');
//     print('\tManufacturer: ${sp.manufacturer}');
//     print('\tSerial Number: ${sp.serialNumber}');
//     // print('\tProduct ID: 0x${sp.productId.toRadixString(16)}');
//     // print('\tVendor ID: 0x${sp.vendorId.toRadixString(16)}');
//     sp.dispose();
//   }
// }

class Arduino {
  int timeout = 2000;
  late SerialPort sp;
  late SPI spi;

  Arduino({
    int baudRate = 115200,
    String? port,
    SerialPort? serialPort,
  }) {
    sp = serialPort ?? SerialPort(port);

    var config = SerialPortConfig();
    config.baudRate = baudRate;
    sp.config = config;

    var success = sp.openReadWrite();

    print(success);
    print(SerialPort.lastError);

    spi = SPI(this);
  }

  /// Sets I/O mode of pin
  /// [mode] should be PIN_INPUT or PIN_OUTPUT
  void pinMode(int pin, int mode) {
    if (mode == PIN_INPUT) {
      pin = -pin;
    }
    var cmd = buildCmdString('pm', [pin, mode]);
    sp.write(cmd);
    sp.flush();
  }

  /// Send digitalWrite command to a pin
  void digitalWrite(int pin, int value) {
    if (value == DIGITAL_LOW) {
      pin = -pin;
    }
    var cmd = buildCmdString('dw', [pin]);
    sp.write(cmd);
    sp.flush();
  }

  /// return 1 for HIGH, 0 for LOW
  int digitalRead(int pin) {
    var cmd = buildCmdString('dr', [pin]);
    sp.write(cmd);
    sp.flush();
    return _readLastInt();
  }

  void analogWrite(int pin, int value) {
    var cmd = buildCmdString('aw', [pin, value]);
    sp.write(cmd);
    sp.flush();
  }

  int analogRead(int pin) {
    var cmd = buildCmdString('ar', [pin]);
    sp.write(cmd);
    sp.flush();
    return _readLastInt();
  }

  int _readLastInt() {
    // the analog value can only be up to 4095?
    var bytes = 6;
    Uint8List buf = sp.read(bytes);
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
