import 'dart:convert';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/comparison_block.dart';
import 'package:robot_router/custom_blocks/if_block.dart';
import 'package:robot_router/custom_blocks/literal_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';
import 'package:robot_router/custom_blocks/while_block.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

class _BlockReturnValue {
  factory _BlockReturnValue.boolean(bool value) =>
      _BlockReturnValue(true, false, false, value, null, null);

  factory _BlockReturnValue.number(double value) =>
      _BlockReturnValue(false, true, false, null, value, null);

  factory _BlockReturnValue.string(String value) =>
      _BlockReturnValue(false, false, true, null, null, value);

  _BlockReturnValue(this.hasConditionalValue, this.hasNumberValue,
      this.hasStringValue, this.boolVal, this.numVal, this.stringVal);

  final bool hasConditionalValue;
  final bool hasNumberValue;
  final bool hasStringValue;

  final bool? boolVal;
  final double? numVal;
  final String? stringVal;
}

class Runtime extends BlockVisitor<_BlockReturnValue> {
  factory Runtime.the() => _instance;
  Runtime._internal();

  static final Runtime _instance = Runtime._internal();
  final WebSocketChannel _channel = WebSocketChannel.connect(
    Uri.parse('ws://79.249.140.33:9090'),
  );

  void exec(List<Block> blocks) {
    for (Block block in blocks) {
      block.accept(this);
    }
  }

  @override
  _BlockReturnValue visitCommentBlock(CommentBlock commentBlock) {
    print('Exec comment Block');
    _channel.sink.add(jsonEncode({
      'op': 'subscribe',
      'topic': 'sonar_range',
      'type': 'sensor_msgs/Range'
    }));

    _channel.stream.listen(
      (dynamic data) => print(data),
      onError: (dynamic error) => print(error),
    );
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitTestBlock(TestBlock testBlock) {
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitWhileBlock(WhileBlock whileBlock) {
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitIfBlock(IfBlock whileBlock) {
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitComparisonBlock(ComparisonBlock comparisonBlock) {
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitLiteralBlock(LiteralBlock literalBlock) {
    return _BlockReturnValue.boolean(false);
  }
}
