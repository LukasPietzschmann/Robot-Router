import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/selectable_block_widget.dart';
import 'package:robot_router/input_provider.dart';

class LiteralBlock extends Block {
  double? literal;

  @override
  T accept<T>(BlockVisitor<T> visitor) => visitor.visitLiteralBlock(this);

  @override
  BlockView<LiteralBlock> construct() =>
      LiteralBlockView(block: this, key: ObjectKey(this));

  @override
  String get name => 'Literal';

  @override
  BlockType get type => BlockType.t_expr;
}

class LiteralBlockView extends BlockView<LiteralBlock> {
  const LiteralBlockView({required LiteralBlock block, required Key key})
      : super(block: block, key: key);

  @override
  State<StatefulWidget> createState() => LiteralBlockState();
}

class LiteralBlockState extends BlockViewState<LiteralBlockView> {
  double? literal;

  @override
  void initState() {
    super.initState();
    literal = widget.block.literal;
  }

  @override
  Widget render() {
    return NumberProvider(onSubmit: (int num) {
      setState(() {
        literal = num.toDouble();
      });
      widget.block.literal = num.toDouble();
    });
  }
}
