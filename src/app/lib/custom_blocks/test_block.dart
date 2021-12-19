import 'package:flutter/material.dart';

import '../block_view.dart';

class TestBlock extends Block {
  const TestBlock({Key? key}) : super(key: key);

  @override
  BlockState createState() => TestBlockState();

  @override
  String get name => 'Test Block';
  @override
  BlockType get type => BlockType.t_action;

  @override
  T accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitTestBlock(this);
  }
}

class TestBlockState extends BlockState {
  @override
  Widget render() {
    return TextButton(
        onPressed: () {
          isCurrentlyRunning = !isCurrentlyRunning;
        },
        child: const Text('Click me'));
  }
}
