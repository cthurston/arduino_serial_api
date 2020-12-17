// @dart=2.9
import 'package:arduino_serial_api/arduino_serial_api.dart';
import 'package:test/test.dart';

void main() {
  test('buildCmdString ', () {
    expect(buildCmdString('version'),
        [64, 118, 101, 114, 115, 105, 111, 110, 37, 36, 33]);
  });

  test('find port', () {
    findPort();
  });

  group('arduino', () {
    Arduino board;
    setUpAll(() async {
      board = Arduino();
      await board.start();
    });
    test('read a value', () {
      var v = board.analogRead(0);
      expect(v, greaterThan(0));
    });

    test('write a value', () {
      board.analogWrite('DAC0', 200);
      board.analogWrite(13, 200);
    });
  });
}
