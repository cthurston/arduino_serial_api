// @dart=2.9
import 'dart:io';
import 'package:dart_serial_port/dart_serial_port.dart';

void main(List<String> arguments) {
  print('Hello world!');
  print('Available ports:');

  var i = 0;

  final name = SerialPort.availablePorts.last;
  final port = SerialPort(name);
  if (!port.openReadWrite()) {
    print(SerialPort.lastError);
    exit(-1);
  }

  final reader = SerialPortReader(port);
  reader.stream.listen((data) {
    var s = String.fromCharCodes(data);
    print('received $s');
  });

  for (final name in SerialPort.availablePorts) {
    final sp = SerialPort(name);
    print('${++i}) $name');
    print('\tDescription: ${sp.description}');
    print('\tManufacturer: ${sp.manufacturer}');
    print('\tSerial Number: ${sp.serialNumber}');
    // print('\tProduct ID: 0x${sp.productId.toRadixString(16)}');
    // print('\tVendor ID: 0x${sp.vendorId.toRadixString(16)}');
    sp.dispose();
  }
}
