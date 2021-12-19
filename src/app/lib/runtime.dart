import 'package:robot_router/block_view.dart';
import 'package:robot_router/custom_blocks/block_with_subblock.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';

class Runtime extends BlockVisitor<void> {
  factory Runtime.the() => _instance;
  Runtime._internal() {
    //init
  }

  static final Runtime _instance = Runtime._internal();

  void exec(List<Block> blocks) {
    for (Block block in blocks) {
      block.accept(this);
    }
  }

  @override
  void visitBlockWithSubblock(BlockWithSubblock blockWithSubblock) {
    print('Exec block with subblock');
    //blockWithSubblock.subblock.accept(this);
  }

  @override
  void visitCommentBlock(CommentBlock commentBlock) {
    print('Exec comment Block');
  }

  @override
  void visitTestBlock(TestBlock testBlock) {
    print('Exec test block');
  }
}
