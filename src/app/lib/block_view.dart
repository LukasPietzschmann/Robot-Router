import 'package:flutter/material.dart';
import 'package:robot_router/custom_blocks/comment_block.dart';
import 'package:robot_router/custom_blocks/comparison_block.dart';
import 'package:robot_router/custom_blocks/drive_in_direction_block.dart';
import 'package:robot_router/custom_blocks/get_distance_block.dart';
import 'package:robot_router/custom_blocks/literal_block.dart';
import 'package:robot_router/custom_blocks/move_head_block.dart';
import 'package:robot_router/custom_blocks/test_block.dart';
import 'package:robot_router/custom_blocks/turn_in_direction_block.dart';
import 'package:robot_router/custom_blocks/while_block.dart';
import 'package:sticky_grouped_list/sticky_grouped_list.dart';

import 'custom_blocks/if_block.dart';

enum BlockType { t_action, t_passive, t_event, t_control, t_expr }

Future<Block?> selectBlockFromList(BuildContext context,
    {List<BlockType> desiredTypes = const <BlockType>[
      BlockType.t_action,
      BlockType.t_passive,
      BlockType.t_event,
      BlockType.t_control,
      BlockType.t_expr
    ]}) {
  return showModalBottomSheet<Block>(
    backgroundColor: Colors.white,
    context: context,
    isDismissible: true,
    enableDrag: false,
    shape: ContinuousRectangleBorder(borderRadius: BorderRadius.circular(15)),
    builder: (BuildContext context) {
      return Padding(
        padding: const EdgeInsets.all(20),
        child: StickyGroupedListView<Block, BlockType>(
          floatingHeader: false,
          stickyHeaderBackgroundColor: Colors.white,
          elements: Block.allBlocks()
              .where((Block block) => desiredTypes.contains(block.type))
              .toList(),
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
  Future<T> visitTestBlock(TestBlock testBlock);
  Future<T> visitCommentBlock(CommentBlock commentBlock);
  Future<T> visitWhileBlock(WhileBlock whileBlock);
  Future<T> visitIfBlock(IfBlock ifBlock);
  Future<T> visitComparisonBlock(ComparisonBlock comparisonBlock);
  Future<T> visitLiteralBlock(LiteralBlock literalBlock);
  Future<T> visitDriveInDirectionBlock(
      DriveInDirectionBlock driveInDirectionBlock);
  Future<T> visitMoveHeadBlock(MoveHeadBlock moveHeadBlock);
  Future<T> visitGetDistanceBlock(GetDistanceBlock getDistanceBlock);
  Future<T> visitTurnInDirectionBlock(
      TurnInDirectionBlock turnInDirectionBlock);
}

abstract class Block {
  String get name;
  BlockType get type;
  String get typeName {
    return <String>[
      'Action Block',
      'Passive Block',
      'Event Block',
      'Control Block',
      'Expression Block'
    ][type.index];
  }

  static List<Block> allBlocks() {
    return <Block>[
      TestBlock(),
      CommentBlock(),
      WhileBlock(),
      IfBlock(),
      ComparisonBlock(),
      LiteralBlock(),
      DriveInDirectionBlock(),
      MoveHeadBlock(),
      GetDistanceBlock(),
      TurnInDirectionBlock()
    ];
  }

  BlockView construct();
  Future<T> accept<T>(BlockVisitor<T> visitor);
}

abstract class BlockView<B extends Block> extends StatefulWidget {
  const BlockView({required this.block, required Key key}) : super(key: key);
  final B block;
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
        borderRadius: BorderRadius.circular(15),
      ),
      child: Padding(padding: const EdgeInsets.all(15), child: render()),
    );
  }
}
