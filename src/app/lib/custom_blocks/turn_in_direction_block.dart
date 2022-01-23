import 'package:flutter/material.dart';

import '../block_view.dart';
import '../input_provider.dart';

enum TurnDirection { right, left }

class TurnInDirectionBlock extends Block {
  TurnDirection? direction;
  int? steps;

  @override
  String get name => 'Turn in direction';

  @override
  BlockType get type => BlockType.t_action;

  @override
  BlockView<Block> construct() =>
      TurnInDirectionBlockView(block: this, key: ObjectKey(this));

  @override
  Future<T> accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitTurnInDirectionBlock(this);
  }
}

class TurnInDirectionBlockView extends BlockView<TurnInDirectionBlock> {
  const TurnInDirectionBlockView(
      {required TurnInDirectionBlock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<TurnInDirectionBlockView> createState() =>
      TurnInDirectionBlockState();
}

class TurnInDirectionBlockState
    extends BlockViewState<TurnInDirectionBlockView> {
  TurnDirection? direction;
  int? steps;

  @override
  void initState() {
    super.initState();
    direction = widget.block.direction;
    steps = widget.block.steps;
  }

  @override
  Widget render() {
    return Row(
      children: <Widget>[
        const Text('Turn '),
        NumberProvider(onSubmit: (int num) {
          setState(() {
            steps = num;
          });
          widget.block.steps = num;
        }),
        const Text(' steps '),
        CappedValueProvider(
            possibleValues: <String>[
              TurnDirection.left.name,
              TurnDirection.right.name
            ],
            onSubmit: (String selected) {
              setState(() {
                if (selected == TurnDirection.left.name) {
                  direction = TurnDirection.left;
                } else {
                  direction = TurnDirection.right;
                }
              });
              widget.block.direction = direction;
            }),
        Text(direction?.name ?? '')
      ],
    );
  }
}
