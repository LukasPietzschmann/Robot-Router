import 'package:flutter/material.dart';

import '../block_view.dart';
import '../input_provider.dart';

enum Motion { Pan, Tilt }

class MoveHeadBlock extends Block {
  Motion? motion;
  int? degrees;

  @override
  String get name => 'Move head';

  @override
  BlockType get type => BlockType.t_action;

  @override
  BlockView<Block> construct() =>
      MoveHeadBlockView(block: this, key: ObjectKey(this));

  @override
  Future<T> accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitMoveHeadBlock(this);
  }
}

class MoveHeadBlockView extends BlockView<MoveHeadBlock> {
  const MoveHeadBlockView({required MoveHeadBlock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<MoveHeadBlockView> createState() => MoveHeadBlockState();
}

class MoveHeadBlockState extends BlockViewState<MoveHeadBlockView> {
  Motion? motion;
  int? degrees;

  @override
  void initState() {
    super.initState();
    motion = widget.block.motion;
    degrees = widget.block.degrees;
  }

  @override
  Widget render() {
    return Row(
      children: <Widget>[
        CappedValueProvider(
            possibleValues: <String>[
              Motion.Pan.name,
              Motion.Tilt.name,
            ],
            onSubmit: (String selected) {
              setState(() {
                if (selected == Motion.Pan.name) {
                  motion = Motion.Pan;
                } else {
                  motion = Motion.Tilt;
                }
              });
              widget.block.motion = motion;
            }),
        Text(motion?.name ?? ''),
        const Text(' Head '),
        NumberProvider(onSubmit: (int number) {
          setState(() {
            degrees = number;
          });
          widget.block.degrees = number;
        }),
        const Text('Degrees')
      ],
    );
  }
}
