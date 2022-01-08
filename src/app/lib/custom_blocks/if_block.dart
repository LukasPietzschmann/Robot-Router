import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/selectable_block_widget.dart';

class IfBlock extends Block {
  Block? condBlock;
  Block? thenBlock;
  Block? elseBlock;

  @override
  T accept<T>(BlockVisitor<T> visitor) => visitor.visitIfBlock(this);

  @override
  BlockView<IfBlock> construct() =>
      IfBlockView(block: this, key: ObjectKey(this));

  @override
  String get name => 'If Else';

  @override
  BlockType get type => BlockType.t_control;
}

class IfBlockView extends BlockView<IfBlock> {
  const IfBlockView({required IfBlock block, required Key key})
      : super(block: block, key: key);

  @override
  State<StatefulWidget> createState() => IfBlockState();
}

class IfBlockState extends BlockViewState<IfBlockView> {
  Block? condBlock;
  Block? thenBlock;
  Block? elseBlock;

  @override
  void initState() {
    super.initState();
    condBlock = widget.block.condBlock;
    thenBlock = widget.block.thenBlock;
    elseBlock = widget.block.elseBlock;
  }

  @override
  Widget render() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        Row(
          mainAxisAlignment: MainAxisAlignment.start,
          children: <Widget>[
            const Text('if'),
            Expanded(
              child: SelectableBlockWidget(
                  hint: 'Hold to select a condition',
                  desiredTypes: const <BlockType>[BlockType.t_expr],
                  setBlock: (Block? block) {
                    setState(() {
                      condBlock = block ?? condBlock;
                    });
                    widget.block.condBlock = block ?? condBlock;
                  },
                  blockToRender: condBlock),
            ),
          ],
        ),
        const Text('then'),
        Padding(
          padding: const EdgeInsets.only(left: 50),
          child: SelectableBlockWidget(
              setBlock: (Block? block) {
                setState(() {
                  thenBlock = block ?? thenBlock;
                });
                widget.block.thenBlock = block ?? thenBlock;
              },
              blockToRender: thenBlock),
        ),
        const Text('else'),
        Padding(
          padding: const EdgeInsets.only(left: 50),
          child: SelectableBlockWidget(
              setBlock: (Block? block) {
                setState(() {
                  elseBlock = block ?? elseBlock;
                });
                widget.block.elseBlock = block ?? elseBlock;
              },
              blockToRender: elseBlock),
        ),
      ],
    );
  }
}
