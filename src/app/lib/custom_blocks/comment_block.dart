import 'package:flutter/material.dart';

import '../block_view.dart';
import '../input_provider.dart';

class CommentBlock extends Block {
  const CommentBlock({this.defaultComment = '', Key? key}) : super(key: key);

  final String defaultComment;

  @override
  String get name => 'Comment Block';

  @override
  BlockState createState() => CommentBlockState();
  @override
  BlockType get type => BlockType.t_passive;

  @override
  T accept<T>(BlockVisitor<T> visitor) {
    return visitor.visitCommentBlock(this);
  }
}

class CommentBlockState extends BlockState {
  @override
  Widget render() {
    return TextProvider(
        onSubmit: (String text) {}, hintText: 'Here goes your Comment');
  }
}
