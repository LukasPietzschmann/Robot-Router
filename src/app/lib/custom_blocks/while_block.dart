import 'package:flutter/material.dart';
import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/selectable_block_widget.dart';

class WhileBlock extends Block {
  Block? thenBlock;
  Block? condBlock;

  @override
  Future<T> accept<T>(BlockVisitor<T> visitor) => visitor.visitWhileBlock(this);

  @override
  BlockView<WhileBlock> construct() =>
      WhileBlockView(block: this, key: ObjectKey(this));

  @override
  String get name => 'While Loop';

  @override
  BlockType get type => BlockType.t_control;
}

class WhileBlockView extends BlockView<WhileBlock> {
  const WhileBlockView({required WhileBlock block, required Key key})
      : super(block: block, key: key);

  @override
  State<StatefulWidget> createState() => WhileBlockState();
}

class WhileBlockState extends BlockViewState<WhileBlockView> {
  Block? thenBlock;
  Block? condBlock;

  @override
  void initState() {
    super.initState();
    thenBlock = widget.block.thenBlock;
    condBlock = widget.block.condBlock;
  }

  @override
  Widget render() {
    return Column(
      crossAxisAlignment: CrossAxisAlignment.stretch,
      children: <Widget>[
        Row(
          children: <Widget>[
            const Text('while'),
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
            )
          ],
        ),
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
        )
      ],
    );
  }
}
