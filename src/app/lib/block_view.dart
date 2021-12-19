import 'package:flutter/material.dart';
import 'package:robot_router/custom_blocks/block_with_subblock.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';
import 'package:sticky_grouped_list/sticky_grouped_list.dart';

enum BlockType { t_action, t_passive, t_event, t_control }

Future<Block?> selectBlockFromList(BuildContext context) {
  return showModalBottomSheet<Block>(
    backgroundColor: Colors.white,
    context: context,
    isDismissible: true,
    enableDrag: false,
    shape: ContinuousRectangleBorder(
      borderRadius: BorderRadius.circular(20),
    ),
    builder: (BuildContext context) {
      return Padding(
        padding: const EdgeInsets.all(20),
        child: StickyGroupedListView<Block, BlockType>(
          floatingHeader: false,
          stickyHeaderBackgroundColor: Colors.white,
          elements: Block.allBlocks(),
          groupBy: (Block block) => block.type,
          groupComparator: (BlockType type1, BlockType type2) =>
              type2.index.compareTo(type1.index),
          itemComparator: (Block block1, Block block2) =>
              block2.name.compareTo(block1.name),
          groupSeparatorBuilder: (Block block) {
            return Padding(
              padding: const EdgeInsets.only(top: 10),
              child: Text(block.typeName,
                  textAlign: TextAlign.center,
                  style: const TextStyle(
                      fontWeight: FontWeight.bold, fontSize: 20)),
            );
          },
          itemBuilder: (BuildContext context, Block block) {
            return TextButton(
                onPressed: () {
                  Navigator.pop(context, block);
                },
                child: Text(block.name),
                style: ButtonStyle(
                    alignment: Alignment.center,
                    padding: MaterialStateProperty.all(const EdgeInsets.all(0)),
                    foregroundColor: MaterialStateProperty.all(Colors.black),
                    visualDensity: const VisualDensity(vertical: -3)));
          },
        ),
      );
    },
  );
}

abstract class BlockVisitor<T> {
  T visitTestBlock(TestBlockView testBlock);
  T visitCommentBlock(CommentBlockView commentBlock);
  T visitBlockWithSubblock(BlockWithSubblockView blockWithSubblock);
}

abstract class Block {
  String get name;
  BlockType get type;
  String get typeName {
    return <String>[
      'Action Block',
      'Passive Block',
      'Event Block',
      'Control Block'
    ][type.index];
  }

  static List<Block> allBlocks() {
    return <Block>[BlockWithSubblock(), TestBlock(), CommentBlock()];
  }

  BlockView construct();
}

abstract class BlockView<B extends Block> extends StatefulWidget {
  const BlockView({required this.block, required Key key}) : super(key: key);
  final B block;

  T accept<T>(BlockVisitor<T> visitor);
}

abstract class BlockViewState<B extends BlockView> extends State<B>
    with AutomaticKeepAliveClientMixin {
  bool _isCurrentlyRunning = false;

  bool get isCurrentlyRunning {
    return _isCurrentlyRunning;
  }

  set isCurrentlyRunning(bool newVal) {
    setState(() {
      _isCurrentlyRunning = newVal;
    });
  }

  Widget render();

  @override
  bool get wantKeepAlive => true;

  @override
  Widget build(BuildContext context) {
    super.build(context);
    return Card(
      margin: const EdgeInsets.all(10),
      shape: ContinuousRectangleBorder(
        side: BorderSide(
            color: _isCurrentlyRunning ? Colors.green : Colors.black, width: 1),
        borderRadius: BorderRadius.circular(20),
      ),
      child: Padding(padding: const EdgeInsets.all(15), child: render()),
    );
  }
}
