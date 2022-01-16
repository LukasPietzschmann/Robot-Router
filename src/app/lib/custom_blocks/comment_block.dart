import 'package:flutter/material.dart';

import '../block_view.dart';
import '../input_provider.dart';

class CommentBlock extends Block {
  String comment = '';

  @override
  String get name => 'Comment Block';

  @override
  BlockType get type => BlockType.t_passive;

  @override
  BlockView<Block> construct() =>
      CommentBlockView(block: this, key: ObjectKey(this));

  @override
  Future<T> accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitCommentBlock(this);
  }
}

class CommentBlockView extends BlockView<CommentBlock> {
  const CommentBlockView({required CommentBlock block, required Key key})
      : super(block: block, key: key);

  @override
  BlockViewState<CommentBlockView> createState() => CommentBlockState();
}

class CommentBlockState extends BlockViewState<CommentBlockView> {
  @override
  Widget render() {
    return TextProvider(
        onSubmit: (String text) => widget.block.comment = text,
        hintText: 'Here goes your Comment');
  }
}
