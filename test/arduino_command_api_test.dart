// @dart=2.9
import 'package:arduino_command_api/arduino_command_api.dart';
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
}
