import 'dart:math';

import 'package:flutter/material.dart';

import '../block_view.dart';

class GetDistanceBlock extends Block {
  @override
  String get name => 'Get distance';

  @override
  BlockType get type => BlockType.t_expr;

  @override
  BlockView<Block> construct() =>
      GetDistanceBlockView(block: this, key: ObjectKey(this));

  @override
  Future<T> accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitGetDistanceBlock(this);
  }
}

class GetDistanceBlockView extends BlockView<GetDistanceBlock> {
  const GetDistanceBlockView(
      {required GetDistanceBlock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<GetDistanceBlockView> createState() => GetDistanceBlockState();
}

class GetDistanceBlockState extends BlockViewState<GetDistanceBlockView> {
  @override
  Widget render() {
    return const Icon(Icons.radar_rounded);
  }
}
