import 'package:flutter/material.dart';

import '../block_view.dart';

class TestBlock extends Block {
  @override
  String get name => 'Test Block';

  @override
  BlockType get type => BlockType.t_passive;

  @override
  BlockView<Block> construct() =>
      TestBlockView(block: this, key: ObjectKey(this));

  @override
  T accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitTestBlock(this);
  }
}

class TestBlockView extends BlockView<TestBlock> {
  const TestBlockView({required TestBlock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<TestBlockView> createState() => TestBlockState();
}

class TestBlockState extends BlockViewState<TestBlockView> {
  @override
  Widget render() {
    return TextButton(
        onPressed: () {
          isCurrentlyRunning = !isCurrentlyRunning;
        },
        child: const Text('Click me'));
  }
}
