// @dart=2.9
import 'package:arduino_serial_api/arduino_serial_api.dart';
import 'package:test/test.dart';

void main() {
  test('buildCmdString ', () {
    expect(buildCmdString('version'), '@version%\$!');
    expect(
        buildCmdString('version', [4, 5.1, 'test']), '@version%4%5.1%test\$!');
  });

  test('arduino', () async {
    var port = '/dev/cu.usbmodem141401';
    var a = Arduino();
    await a.start();
  });

  test('find port', () {
    findPort();
  });
}