import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/selectable_block_widget.dart';
import 'package:robot_router/input_provider.dart';

class ComparisonBlock extends Block {
  Block? lhs;
  String? selectedOper;
  Block? rhs;

  @override
  Future<T> accept<T>(BlockVisitor<T> visitor) =>
      visitor.visitComparisonBlock(this);

  @override
  BlockView<ComparisonBlock> construct() =>
      ComparisonBlockView(block: this, key: ObjectKey(this));

  @override
  String get name => 'Comparison';

  @override
  BlockType get type => BlockType.t_expr;
}

class ComparisonBlockView extends BlockView<ComparisonBlock> {
  const ComparisonBlockView({required ComparisonBlock block, required Key key})
      : super(block: block, key: key);

  @override
  State<StatefulWidget> createState() => ComparisonBlockState();
}

class ComparisonBlockState extends BlockViewState<ComparisonBlockView> {
  Block? lhs;
  String? selectedOper;
  Block? rhs;

  @override
  void initState() {
    super.initState();
    lhs = widget.block.lhs;
    selectedOper = widget.block.selectedOper;
    rhs = widget.block.rhs;
  }

  @override
  Widget render() {
    return Row(
      mainAxisAlignment: MainAxisAlignment.spaceAround,
      children: <Widget>[
        Expanded(
          child: SelectableBlockWidget(
              hint: '  ',
              desiredTypes: const <BlockType>[BlockType.t_expr],
              setBlock: (Block? block) {
                setState(() {
                  lhs = block ?? lhs;
                });
                widget.block.lhs = block ?? lhs;
              },
              blockToRender: lhs),
        ),
        Expanded(
          child: CappedValueProvider(
              possibleValues: const <String>['<', '<=', '>', '>=', '==', '!='],
              onSubmit: (String submitted) {
                setState(() {
                  selectedOper = submitted;
                });
                widget.block.selectedOper = submitted;
              }),
        ),
        Text(selectedOper ?? ''),
        Expanded(
          child: SelectableBlockWidget(
              hint: '  ',
              desiredTypes: const <BlockType>[BlockType.t_expr],
              setBlock: (Block? block) {
                setState(() {
                  rhs = block ?? rhs;
                });
                widget.block.rhs = block ?? rhs;
              },
              blockToRender: rhs),
        ),
      ],
    );
  }
}
