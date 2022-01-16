import 'dart:convert';

import 'package:web_socket_channel/web_socket_channel.dart';

class Topic {
  Topic(this._rosbridge, this._name, this._messageType,
      [this._isService = false]);

  final Rosbridge _rosbridge;
  final String _name;
  final String _messageType;
  final bool _isService;

  void subscribe(Function(dynamic data) callback) {
    _rosbridge._subscribe(
        _name, _messageType, (dynamic data) => callback(data));
  }

  Future<dynamic> subscribeAndGetFirst() async {
    return _rosbridge._subscribeAndGetFirst(_name, _messageType);
  }

  void publish(Object message) {
    _rosbridge._publish(_name, _messageType, message, _isService);
  }

  void advertise() {} //op: advertise
}

class Rosbridge {
  Rosbridge();

  void connect(String ip, int port,
      [Function(WebSocketChannelException e)? onError]) {
    //if (_connected) return;
    try {
      _channel = WebSocketChannel.connect(
          Uri.parse('ws://' + ip + ':' + port.toString()));
      _connected = true;
    } catch (e) {
      _connected = false;
      onError!(e as WebSocketChannelException);
    }
  }

  late WebSocketChannel _channel;
  bool _connected = false;

  bool _subscribe(String topic, String type, Function(dynamic data) callback) {
    if (!_connected) return false;
    _channel.sink
        .add(jsonEncode({'op': 'subscribe', 'topic': topic, 'type': type}));
    _channel.stream
        .listen((dynamic data) => callback(data), cancelOnError: true);
    return true;
  }

  Future _subscribeAndGetFirst(String topic, String type) {
    if (!_connected) return Future.error('Not connected');
    _channel.sink
        .add(jsonEncode({'op': 'subscribe', 'topic': topic, 'type': type}));
    return _channel.stream.first;
  }

  bool _publish(String topic, String type, Object message,
      [bool isService = false]) {
    if (!_connected) return false;
    if (!isService) {
      _channel.sink.add(jsonEncode(
          {'op': 'publish', 'topic': topic, 'type': type, 'msg': message}));
    } else {
      _channel.sink.add(jsonEncode({
        'op': 'call_service',
        'service': topic,
        'type': type,
        'args': message
      }));
    }
    return true;
  }

  void closeConnection([int closeCode = 1000]) {
    if (!_connected) return;
    _channel.sink.close(closeCode);
    _connected = false;
  }
}
