import 'package:flutter/material.dart';

import '../block_view.dart';

class CommentBlock extends Block {
  const CommentBlock({this.defaultComment = '', Key? key}) : super(key: key);

  final String defaultComment;

  @override
  String get name => 'Comment Block';

  @override
  BlockState createState() => CommentBlockState();
  @override
  BlockType get type => BlockType.t_passive;
}

class CommentBlockState extends BlockState {
  @override
  Widget render() {
    return TextProvider(
        onSubmit: (String text) {}, hintText: 'Here goes your Comment');
  }
}
