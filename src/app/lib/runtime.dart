import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/comparison_block.dart';
import 'package:robot_router/custom_blocks/drive_in_direction_block.dart';
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
  _BlockReturnValue visitCommentBlock(CommentBlock commentBlock) {
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

  @override
  _BlockReturnValue visitDriveInDirectionBlock(
      DriveInDirectionBlock driveInDirectionBlock) {
    late Topic t;
    if (driveInDirectionBlock.direction == Direction.forward) {
      t = Topic(rb, '/motor_drive_forward', 'awesom_o_robot/MotorServiceValues',
          true);
    } else {
      t = Topic(rb, '/motor_drive_backward',
          'awesom_o_robot/MotorServiceValues', true);
    }
    t.publish({'speed': 100, 'duration': driveInDirectionBlock.steps! / 2});
    return _BlockReturnValue.boolean(false);
  }

  @override
  _BlockReturnValue visitMoveHeadBlock(MoveHeadBlock moveHeadBlock) {
    late Topic t;
    if (moveHeadBlock.motion == Motion.Pan) {
      t = Topic(rb, '/pan_servo_angle', '/std_msgs/Int16');
    } else {
      t = Topic(rb, '/tilt_servo_angle', '/std_msgs/Int16');
    }
    t.publish({'data': moveHeadBlock.degrees});
    return _BlockReturnValue.boolean(false);
  }
}
