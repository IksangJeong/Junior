"""
데이터베이스 연습 #310
SQLite CRUD 실습
"""
import sqlite3

def setup_310():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_310 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_310_0', 667.45, 'A'), ('item_310_1', 764.24, 'C'), ('item_310_2', 757.19, 'A'), ('item_310_3', 530.25, 'C'), ('item_310_4', 706.73, 'A')]
    ]
    cursor.executemany(
        'INSERT INTO items_310 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_310()
    cursor = conn.cursor()

    print(f"DB 연습 #310")
    cursor.execute('SELECT * FROM items_310')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_310 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
