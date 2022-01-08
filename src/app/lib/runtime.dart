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
      _BlockReturnValue(hasConditionalValue: true, boolVal: value);

  factory _BlockReturnValue.number(double value) =>
      _BlockReturnValue(hasNumberValue: true, numVal: value);

  factory _BlockReturnValue.string(String value) =>
      _BlockReturnValue(hasStringValue: true, stringVal: value);

  _BlockReturnValue(
      {this.hasConditionalValue = false,
      this.hasNumberValue = false,
      this.hasStringValue = false,
      this.boolVal = false,
      this.numVal = -1,
      this.stringVal = ''});

  final bool hasConditionalValue;
  final bool hasNumberValue;
  final bool hasStringValue;

  final bool boolVal;
  final double numVal;
  final String stringVal;
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
    _BlockReturnValue condition = whileBlock.condBlock!.accept(this);
    assert(condition.hasConditionalValue);
    if (condition.boolVal) {
      whileBlock.thenBlock!.accept(this);
    }
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitIfBlock(IfBlock ifBlock) {
    _BlockReturnValue condition = ifBlock.condBlock!.accept(this);
    assert(condition.hasConditionalValue);
    if (condition.boolVal) {
      ifBlock.thenBlock!.accept(this);
    } else {
      ifBlock.elseBlock!.accept(this);
    }
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitComparisonBlock(ComparisonBlock comparisonBlock) {
    _BlockReturnValue lhs = comparisonBlock.lhs!.accept(this);
    _BlockReturnValue rhs = comparisonBlock.rhs!.accept(this);

    switch (comparisonBlock.selectedOper) {
      case '<':
        return _BlockReturnValue.boolean(lhs.numVal < rhs.numVal);
      case '<=':
        return _BlockReturnValue.boolean(lhs.numVal <= rhs.numVal);
      case '>':
        return _BlockReturnValue.boolean(lhs.numVal > rhs.numVal);
      case '>=':
        return _BlockReturnValue.boolean(lhs.numVal >= rhs.numVal);
      case '==':
        return _BlockReturnValue.boolean(lhs.numVal == rhs.numVal);
      case '!=':
        return _BlockReturnValue.boolean(lhs.numVal != rhs.numVal);
      default:
        return _BlockReturnValue.boolean(false);
    }
  }

  @override
  _BlockReturnValue visitLiteralBlock(LiteralBlock literalBlock) {
    return _BlockReturnValue.number(literalBlock.literal!);
  }
}
