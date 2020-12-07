// @dart=2.9
import 'package:arduino_command_api/arduino_command_api.dart';
import 'package:dart_serial_port/dart_serial_port.dart';
import 'package:test/test.dart';

void main() {
  test('buildCmdString ', () {
    expect(buildCmdString('version'), '@version%\$!');
    expect(
        buildCmdString('version', [4, 5.1, 'test']), '@version%4%5.1%test\$!');
  });

  test('arduino', () {
    var port = '/dev/cu.Bluetooth-Incoming-Port';
    var a = Arduino(port: port);
  });

  test('junk', () {
    print('hey\r\n');
    print('heya\r\n'.replaceAll('\r\n', ''));

    print(int.parse('405') + 45);
    var spc = SerialPortConfig();
    spc.baudRate = 112500;
  });
}
