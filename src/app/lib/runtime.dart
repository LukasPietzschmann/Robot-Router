import 'dart:convert';

import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/comparison_block.dart';
import 'package:robot_router/custom_blocks/drive_in_direction_block.dart';
import 'package:robot_router/custom_blocks/get_distance_block.dart';
import 'package:robot_router/custom_blocks/if_block.dart';
import 'package:robot_router/custom_blocks/literal_block.dart';
import 'package:robot_router/custom_blocks/move_head_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';
import 'package:robot_router/custom_blocks/while_block.dart';
import 'package:robot_router/ws_wrapper/rosbridge.dart';

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

  final Rosbridge rb = Rosbridge();

  void exec(List<Block> blocks) {
    rb.connect('87.183.50.244', 9090);
    for (Block block in blocks) {
      block.accept(this);
    }
    //rb.closeConnection();
  }

  @override
  Future<_BlockReturnValue> visitCommentBlock(CommentBlock commentBlock) async {
    return _BlockReturnValue.boolean(false);
  }

  @override
  Future<_BlockReturnValue> visitTestBlock(TestBlock testBlock) async {
    return _BlockReturnValue.boolean(false);
  }

  @override
  Future<_BlockReturnValue> visitWhileBlock(WhileBlock whileBlock) async {
    _BlockReturnValue condition = await whileBlock.condBlock!.accept(this);
    assert(condition.hasConditionalValue);
    print(condition.boolVal ? 'true' : 'false');
    while (condition.boolVal) {
      await whileBlock.thenBlock!.accept(this);
    }
    return _BlockReturnValue.boolean(false);
  }

  @override
  Future<_BlockReturnValue> visitIfBlock(IfBlock ifBlock) async {
    _BlockReturnValue condition = await ifBlock.condBlock!.accept(this);
    assert(condition.hasConditionalValue);
    if (condition.boolVal) {
      ifBlock.thenBlock!.accept(this);
    } else {
      ifBlock.elseBlock!.accept(this);
    }
    return _BlockReturnValue.boolean(false);
  }

  @override
  Future<_BlockReturnValue> visitComparisonBlock(
      ComparisonBlock comparisonBlock) async {
    _BlockReturnValue lhs = await comparisonBlock.lhs!.accept(this);
    _BlockReturnValue rhs = await comparisonBlock.rhs!.accept(this);
    print('lhs: ' + lhs.numVal.toString() + ' rhs: ' + rhs.numVal.toString());

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
  Future<_BlockReturnValue> visitLiteralBlock(LiteralBlock literalBlock) async {
    return _BlockReturnValue.number(literalBlock.literal ?? 0);
  }

  @override
  Future<_BlockReturnValue> visitDriveInDirectionBlock(
      DriveInDirectionBlock driveInDirectionBlock) async {
    print("hiii");
    late Topic t;
    if (driveInDirectionBlock.direction == Direction.forward) {
      t = Topic(rb, '/motor_drive_forward', 'awesom_o_robot/MotorServiceValues',
          true);
    } else {
      t = Topic(rb, '/motor_drive_backward',
          'awesom_o_robot/MotorServiceValues', true);
    }
    t.publish({
      'speed': 100,
      'duration': (driveInDirectionBlock.steps!).round() * 1000
    });
    await Future.delayed(
        Duration(seconds: (driveInDirectionBlock.steps!).round()));
    return _BlockReturnValue.boolean(false);
  }

  @override
  Future<_BlockReturnValue> visitMoveHeadBlock(
      MoveHeadBlock moveHeadBlock) async {
    late Topic t;
    if (moveHeadBlock.motion == Motion.Pan) {
      t = Topic(rb, '/pan_servo_angle', '/std_msgs/Int16');
    } else {
      t = Topic(rb, '/tilt_servo_angle', '/std_msgs/Int16');
    }
    print(moveHeadBlock.degrees);
    t.publish({'data': moveHeadBlock.degrees});
    await Future.delayed(const Duration(milliseconds: 500));
    return _BlockReturnValue.boolean(false);
  }

  @override
  Future<_BlockReturnValue> visitGetDistanceBlock(
      GetDistanceBlock getDistanceBlock) async {
    String res = await Topic(rb, '/sonar_range', 'sensor_msgs/Range')
        .subscribeAndGetFirst();
    return _BlockReturnValue.number(jsonDecode(res)['msg']['range'] as double);
  }
}
