import 'package:flutter/material.dart';

import '../block_view.dart';
import '../input_provider.dart';

enum Direction { forward, backward }

class DriveInDirectionBlock extends Block {
  Direction? direction;
  int? steps;

  @override
  String get name => 'Drive in direction Block';

  @override
  BlockType get type => BlockType.t_action;

  @override
  BlockView<Block> construct() =>
      DriveInDirectionBlockView(block: this, key: ObjectKey(this));

  @override
  T accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitDriveInDirectionBlock(this);
  }
}

class DriveInDirectionBlockView extends BlockView<DriveInDirectionBlock> {
  const DriveInDirectionBlockView(
      {required DriveInDirectionBlock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<DriveInDirectionBlockView> createState() =>
      DriveInDirectionBlockState();
}

class DriveInDirectionBlockState
    extends BlockViewState<DriveInDirectionBlockView> {
  Direction? direction;
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
        const Text('Drive '),
        NumberProvider(onSubmit: (int num) {
          setState(() {
            steps = num;
          });
          widget.block.steps = num;
        }),
        const Text(' steps '),
        CappedValueProvider(
            possibleValues: <String>[
              Direction.forward.name,
              Direction.backward.name
            ],
            onSubmit: (String selected) {
              setState(() {
                if (selected == Direction.forward.name) {
                  direction = Direction.forward;
                } else {
                  direction = Direction.backward;
                }
              });
              widget.block.direction = direction;
            }),
        Text(direction?.name ?? '')
      ],
    );
  }
}
